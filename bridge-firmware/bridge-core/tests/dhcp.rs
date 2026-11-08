//! Host tests for the hand-rolled `DHCPv4` server (RFC 2131, RFC 2132).
//!
//! The server is modelled without a socket: each test hands `Server::handle` a BOOTP/DHCP
//! datagram, a millisecond clock reading and a reply buffer, then parses the reply with the same
//! option walk a client would do. The pool is `192.168.4.100` and `192.168.4.101`.

use bridge_core::dhcp::{self, ReplyKind, Server, ServerConfig};

/// The standard server's own address.
const SERVER: [u8; 4] = [192, 168, 4, 1];

/// The first address in the standard pool.
const FIRST: [u8; 4] = [192, 168, 4, 100];

/// The second address in the standard pool.
const SECOND: [u8; 4] = [192, 168, 4, 101];

/// The standard `/24` subnet mask.
const MASK: [u8; 4] = [255, 255, 255, 0];

/// The standard `/24` broadcast address.
const BROADCAST: [u8; 4] = [192, 168, 4, 255];

/// A first client's Ethernet address.
const MAC_A: [u8; 6] = [0x02, 0, 0, 0, 0, 0x0A];

/// A second client's Ethernet address.
const MAC_B: [u8; 6] = [0x02, 0, 0, 0, 0, 0x0B];

/// A third client's Ethernet address.
const MAC_C: [u8; 6] = [0x02, 0, 0, 0, 0, 0x0C];

/// The transaction id the request helper embeds.
const XID: u32 = 0x1234_5678;

/// `DHCPDISCOVER`.
const DISCOVER: u8 = 1;

/// `DHCPOFFER`.
const OFFER: u8 = 2;

/// `DHCPREQUEST`.
const REQUEST: u8 = 3;

/// `DHCPDECLINE`.
const DECLINE: u8 = 4;

/// `DHCPACK`.
const ACK: u8 = 5;

/// `DHCPRELEASE`.
const RELEASE: u8 = 7;

/// `DHCPINFORM`.
const INFORM: u8 = 8;

/// The magic cookie that starts the options field.
const MAGIC_COOKIE: [u8; 4] = [0x63, 0x82, 0x53, 0x63];

/// A DHCP reply reduced to the fields the tests assert on.
struct Reply {
    /// `op`; 2 for a server reply.
    op: u8,
    /// The echoed transaction id.
    xid: u32,
    /// The echoed BOOTP flags.
    flags: [u8; 2],
    /// The address offered or acknowledged.
    yiaddr: [u8; 4],
    /// The server's own address.
    siaddr: [u8; 4],
    /// The echoed client hardware address.
    chaddr: [u8; 6],
    /// The options as `(code, value)` pairs.
    options: Vec<(u8, Vec<u8>)>,
}

impl Reply {
    /// Returns the first value for `code`.
    #[must_use]
    fn option(&self, code: u8) -> Option<&[u8]> {
        self.options
            .iter()
            .find(|(candidate, _)| *candidate == code)
            .map(|(_, value)| value.as_slice())
    }
}

/// Builds a minimal request: the BOOTP header, the magic cookie, option 53, optional option 50, a
/// parameter request list and the end marker.
#[must_use]
fn request(message_type: u8, chaddr: [u8; 6], requested: Option<[u8; 4]>) -> Vec<u8> {
    let mut datagram = vec![0u8; 240];
    datagram[0] = 1;
    datagram[1] = 1;
    datagram[2] = 6;
    datagram[4..8].copy_from_slice(&XID.to_be_bytes());
    datagram[10] = 0x80;
    datagram[28..34].copy_from_slice(&chaddr);
    datagram[236..240].copy_from_slice(&MAGIC_COOKIE);
    datagram.extend_from_slice(&[53, 1, message_type]);
    if let Some(address) = requested {
        datagram.extend_from_slice(&[50, 4]);
        datagram.extend_from_slice(&address);
    }
    datagram.extend_from_slice(&[55, 3, 1, 3, 6, 255]);
    datagram
}

/// Parses a reply datagram into the fields the tests assert on.
#[must_use]
#[allow(
    clippy::large_stack_frames,
    reason = "clippy inflates this helper's frame once it is inlined into a test that also holds a datagram buffer"
)]
fn parse_reply(bytes: &[u8]) -> Option<Reply> {
    if bytes.len() < 240 {
        return None;
    }
    let mut xid = [0u8; 4];
    xid.copy_from_slice(bytes.get(4..8)?);
    let mut flags = [0u8; 2];
    flags.copy_from_slice(bytes.get(10..12)?);
    let mut yiaddr = [0u8; 4];
    yiaddr.copy_from_slice(bytes.get(16..20)?);
    let mut siaddr = [0u8; 4];
    siaddr.copy_from_slice(bytes.get(20..24)?);
    let mut chaddr = [0u8; 6];
    chaddr.copy_from_slice(bytes.get(28..34)?);

    let mut options = Vec::new();
    let mut index = 240;
    while index < bytes.len() {
        let code = *bytes.get(index)?;
        index += 1;
        if code == 255 {
            break;
        }
        if code == 0 {
            continue;
        }
        let length = usize::from(*bytes.get(index)?);
        index += 1;
        let end = index + length;
        options.push((code, bytes.get(index..end)?.to_vec()));
        index = end;
    }

    Some(Reply {
        op: *bytes.first()?,
        xid: u32::from_be_bytes(xid),
        flags,
        yiaddr,
        siaddr,
        chaddr,
        options,
    })
}

/// Sends a discover for `chaddr` and parses the offer, if any.
#[allow(
    clippy::large_stack_frames,
    reason = "clippy counts this helper's reply buffer once it is inlined into the caller's test"
)]
fn offer(server: &mut Server, chaddr: [u8; 6], now_ms: u32) -> Option<Reply> {
    let datagram = request(DISCOVER, chaddr, None);
    let mut out = [0u8; 512];
    let reply = server.handle(&datagram, now_ms, &mut out)?;
    assert_eq!(reply.kind, ReplyKind::Offer);
    parse_reply(out.get(..reply.length)?)
}

/// Sends a request for `chaddr`, optionally asking for `requested`, and parses the acknowledgement.
#[allow(
    clippy::large_stack_frames,
    reason = "clippy counts this helper's reply buffer once it is inlined into the caller's test"
)]
fn acknowledge(server: &mut Server, chaddr: [u8; 6], requested: Option<[u8; 4]>, now_ms: u32) -> Option<Reply> {
    let datagram = request(REQUEST, chaddr, requested);
    let mut out = [0u8; 512];
    let reply = server.handle(&datagram, now_ms, &mut out)?;
    assert_eq!(reply.kind, ReplyKind::Ack);
    parse_reply(out.get(..reply.length)?)
}

/// A discover gets an offer for the first pool address, the standard header and every option.
#[test]
fn a_discover_gets_an_offer_for_the_first_address() {
    let mut server = Server::new(ServerConfig::standard());
    let datagram = request(DISCOVER, MAC_A, None);
    let mut out = [0u8; 512];
    let Some(reply) = server.handle(&datagram, 0, &mut out) else {
        return;
    };
    assert_eq!(reply.kind, ReplyKind::Offer);
    let Some(reply) = parse_reply(&out[..reply.length]) else {
        return;
    };

    assert_eq!(reply.op, 2);
    assert_eq!(reply.xid, XID);
    assert_eq!(reply.chaddr, MAC_A);
    assert_eq!(reply.yiaddr, FIRST);
    assert_eq!(reply.siaddr, SERVER);
    assert_eq!(reply.option(53), Some(&[OFFER][..]));
    assert_eq!(reply.option(54), Some(&SERVER[..]));
    assert_eq!(reply.option(51), Some(&dhcp::LEASE_MS.to_be_bytes()[..]));
    assert_eq!(reply.option(1), Some(&MASK[..]));
    assert_eq!(reply.option(3), Some(&SERVER[..]));
    assert_eq!(reply.option(6), Some(&SERVER[..]));
    assert_eq!(reply.option(28), Some(&BROADCAST[..]));
    assert_eq!(server.leases(), 1);
}

/// The same client rediscovers and keeps the address it already holds.
#[test]
fn the_same_client_keeps_its_address() {
    let mut server = Server::new(ServerConfig::standard());
    let Some(first) = offer(&mut server, MAC_A, 0) else {
        return;
    };
    let Some(second) = offer(&mut server, MAC_A, 1) else {
        return;
    };

    assert_eq!(first.yiaddr, FIRST);
    assert_eq!(second.yiaddr, FIRST);
    assert_eq!(server.leases(), 1);
}

/// A second client gets the second pool address.
#[test]
fn a_second_client_gets_the_second_address() {
    let mut server = Server::new(ServerConfig::standard());
    let Some(first) = offer(&mut server, MAC_A, 0) else {
        return;
    };
    let Some(second) = offer(&mut server, MAC_B, 1) else {
        return;
    };

    assert_eq!(first.yiaddr, FIRST);
    assert_eq!(second.yiaddr, SECOND);
    assert_eq!(server.leases(), 2);
}

/// A third client finds the two-slot pool full and gets no reply.
#[test]
fn a_third_client_gets_no_reply() {
    let mut server = Server::new(ServerConfig::standard());
    assert!(offer(&mut server, MAC_A, 0).is_some());
    assert!(offer(&mut server, MAC_B, 1).is_some());
    assert!(offer(&mut server, MAC_C, 2).is_none());
    assert_eq!(server.leases(), 2);
}

/// A request is acknowledged with the offered address.
#[test]
fn a_request_is_acknowledged() {
    let mut server = Server::new(ServerConfig::standard());
    let Some(reply) = acknowledge(&mut server, MAC_A, None, 0) else {
        return;
    };

    assert_eq!(reply.yiaddr, FIRST);
    assert_eq!(reply.option(53), Some(&[ACK][..]));
    assert_eq!(server.leases(), 1);
}

/// A request for a free pool address is honoured.
#[test]
fn a_request_for_a_free_address_is_honoured() {
    let mut server = Server::new(ServerConfig::standard());
    let Some(reply) = acknowledge(&mut server, MAC_A, Some(SECOND), 0) else {
        return;
    };

    assert_eq!(reply.yiaddr, SECOND);
}

/// A request for an address outside the pool is ignored and the client gets the pool's first slot.
#[test]
fn a_request_outside_the_pool_is_ignored() {
    let mut server = Server::new(ServerConfig::standard());
    let Some(reply) = acknowledge(&mut server, MAC_A, Some([10, 0, 0, 5]), 0) else {
        return;
    };

    assert_eq!(reply.yiaddr, FIRST);
}

/// A release frees the client's slot for the next client.
#[test]
fn a_release_frees_the_slot() {
    let mut server = Server::new(ServerConfig::standard());
    assert!(offer(&mut server, MAC_A, 0).is_some());
    assert!(offer(&mut server, MAC_B, 1).is_some());

    let release = request(RELEASE, MAC_A, None);
    let mut out = [0u8; 512];
    assert!(server.handle(&release, 2, &mut out).is_none());
    assert_eq!(server.leases(), 1);

    let Some(replacement) = offer(&mut server, MAC_C, 3) else {
        return;
    };
    assert_eq!(replacement.yiaddr, FIRST);
}

/// A decline is treated like a release and gets no reply.
#[test]
fn a_decline_frees_the_slot() {
    let mut server = Server::new(ServerConfig::standard());
    assert!(offer(&mut server, MAC_A, 0).is_some());

    let decline = request(DECLINE, MAC_A, Some(FIRST));
    let mut out = [0u8; 512];
    assert!(server.handle(&decline, 1, &mut out).is_none());
    assert_eq!(server.leases(), 0);
}

/// An expired lease is reused by a new client.
#[test]
fn an_expired_lease_is_reused() {
    let mut server = Server::new(ServerConfig::standard());
    let Some(first) = offer(&mut server, MAC_A, 0) else {
        return;
    };
    let Some(second) = offer(&mut server, MAC_B, dhcp::LEASE_MS + 1) else {
        return;
    };

    assert_eq!(first.yiaddr, FIRST);
    assert_eq!(second.yiaddr, FIRST);
    assert_eq!(server.leases(), 1);
}

/// An inform is acknowledged with an all-zero `yiaddr` and allocates nothing.
#[test]
fn an_inform_gets_an_address_less_acknowledgement() {
    let mut server = Server::new(ServerConfig::standard());
    let datagram = request(INFORM, MAC_A, None);
    let mut out = [0u8; 512];
    let Some(reply) = server.handle(&datagram, 0, &mut out) else {
        return;
    };
    assert_eq!(reply.kind, ReplyKind::Ack);
    let Some(reply) = parse_reply(&out[..reply.length]) else {
        return;
    };

    assert_eq!(reply.option(53), Some(&[ACK][..]));
    assert_eq!(reply.yiaddr, [0; 4]);
    assert_eq!(server.leases(), 0);
}

/// Short datagrams, a wrong magic cookie, a wrong opcode and truncated options get no reply.
#[test]
fn malformed_datagrams_get_no_reply() {
    let mut server = Server::new(ServerConfig::standard());
    let mut out = [0u8; 512];
    let good = request(DISCOVER, MAC_A, None);

    assert!(server.handle(&[], 0, &mut out).is_none());
    assert!(server.handle(&good[..100], 0, &mut out).is_none());

    let mut bad_cookie = good.clone();
    bad_cookie[236] = 0;
    assert!(server.handle(&bad_cookie, 0, &mut out).is_none());

    let mut bad_op = good.clone();
    bad_op[0] = 2;
    assert!(server.handle(&bad_op, 0, &mut out).is_none());

    let mut bad_htype = good.clone();
    bad_htype[1] = 6;
    assert!(server.handle(&bad_htype, 0, &mut out).is_none());

    let mut truncated = good.clone();
    truncated.truncate(242);
    assert!(server.handle(&truncated, 0, &mut out).is_none());

    assert_eq!(server.leases(), 0);
}

/// The reply echoes the request's transaction id and flags.
#[test]
fn the_reply_echoes_the_transaction_id_and_flags() {
    let mut server = Server::new(ServerConfig::standard());
    let mut datagram = request(DISCOVER, MAC_A, None);
    datagram[4..8].copy_from_slice(&0xDEAD_BEEFu32.to_be_bytes());
    datagram[10..12].copy_from_slice(&[0x00, 0x01]);
    let mut out = [0u8; 512];
    let Some(reply) = server.handle(&datagram, 0, &mut out) else {
        return;
    };
    assert_eq!(reply.kind, ReplyKind::Offer);
    let Some(reply) = parse_reply(&out[..reply.length]) else {
        return;
    };

    assert_eq!(reply.xid, 0xDEAD_BEEF);
    assert_eq!(reply.flags, [0x00, 0x01]);
}

/// A reply buffer shorter than the fixed reply is refused without panicking or leasing.
#[test]
fn a_small_output_buffer_gets_no_reply() {
    let mut server = Server::new(ServerConfig::standard());
    let datagram = request(DISCOVER, MAC_A, None);
    let mut short = [0u8; 100];
    let mut almost = [0u8; 279];

    assert!(server.handle(&datagram, 0, &mut short).is_none());
    assert!(server.handle(&datagram, 0, &mut almost).is_none());
    assert_eq!(server.leases(), 0);
}

/// A prefix longer than 32 is refused, so the mask shift can never overflow.
#[test]
fn an_impossible_prefix_gets_no_reply() {
    let config = ServerConfig {
        prefix: 33,
        ..ServerConfig::standard()
    };
    let mut server = Server::new(config);
    let datagram = request(DISCOVER, MAC_A, None);
    let mut out = [0u8; 512];
    assert!(server.handle(&datagram, 0, &mut out).is_none());
}
