//! The AP's hand-rolled `DHCPv4` server (RFC 2131, RFC 2132).
//!
//! The Bridge's soft AP runs a tiny DHCP server so the owner's phone or laptop gets an address
//! without a second daemon: UDP 67 on the server side, UDP 68 on the client side, a
//! `/24` on `192.168.4.1` and a two-address pool. The codec is `no_std` and allocation-free; the
//! caller owns the socket, passes a monotonic millisecond timestamp and supplies the reply buffer.
//!
//! The server answers `DHCPDISCOVER` with `DHCPOFFER`, `DHCPREQUEST` with `DHCPACK`,
//! `DHCPINFORM` with an address-less `DHCPACK`, and treats `DHCPDECLINE` and `DHCPRELEASE` as a
//! signal to drop the matching lease. Malformed datagrams, other message types and a full pool get
//! no reply at all.

/// The DHCP server's UDP port (RFC 2131).
pub const SERVER_PORT: u16 = 67;

/// The DHCP client's UDP port (RFC 2131).
pub const CLIENT_PORT: u16 = 68;

/// The lease handed to clients, in milliseconds (one hour).
pub const LEASE_MS: u32 = 3_600_000;

/// How many clients the built-in pool can lease at once.
pub const POOL_SIZE: usize = 2;

/// The kind of DHCP reply the server produced.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum ReplyKind {
    /// DHCPOFFER, answering a DISCOVER.
    Offer,
    /// DHCPACK, answering a REQUEST or INFORM.
    Ack,
}

/// One DHCP reply: how many bytes were written to the caller's buffer and its message type.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Reply {
    /// The number of bytes written to the output buffer.
    pub length: usize,
    /// The reply's DHCP message type.
    pub kind: ReplyKind,
}

/// The fixed BOOTP header length, before the magic cookie and the options.
const HEADER_LEN: usize = 236;

/// The magic cookie that starts the DHCP options field (RFC 2132 section 3).
const MAGIC_COOKIE: [u8; 4] = [0x63, 0x82, 0x53, 0x63];

/// `op` for a client request (`BOOTREQUEST`).
const OP_REQUEST: u8 = 1;

/// `op` for a server reply (`BOOTREPLY`).
const OP_REPLY: u8 = 2;

/// `htype` for a 10 Mbit Ethernet hardware address.
const HARDWARE_ETHERNET: u8 = 1;

/// `hlen` for a 6-byte Ethernet hardware address.
const HARDWARE_LEN: u8 = 6;

/// The message type option (RFC 2132 section 9.6).
const OPT_MESSAGE_TYPE: u8 = 53;

/// The requested address option (RFC 2132 section 9.1).
const OPT_REQUESTED_IP: u8 = 50;

/// The server identifier option (RFC 2132 section 9.7).
const OPT_SERVER_ID: u8 = 54;

/// The lease time option (RFC 2132 section 9.2).
const OPT_LEASE_TIME: u8 = 51;

/// The subnet mask option (RFC 2132 section 3.3).
const OPT_SUBNET_MASK: u8 = 1;

/// The router option (RFC 2132 section 3.5).
const OPT_ROUTER: u8 = 3;

/// The domain name server option (RFC 2132 section 3.8).
const OPT_DNS: u8 = 6;

/// The broadcast address option (RFC 2132 section 3.4).
const OPT_BROADCAST: u8 = 28;

/// The end-of-options marker (RFC 2132 section 3.1).
const OPT_END: u8 = 255;

/// The one-byte padding marker (RFC 2132 section 3.1).
const OPT_PAD: u8 = 0;

/// `DHCPDISCOVER`: the client looks for a server (RFC 2131 section 4.3.1).
const DISCOVER: u8 = 1;

/// `DHCPOFFER`: the server offers an address (RFC 2131 section 4.3.1).
const OFFER: u8 = 2;

/// `DHCPREQUEST`: the client asks for an offered address (RFC 2131 section 4.3.2).
const REQUEST: u8 = 3;

/// `DHCPDECLINE`: the client rejects an offered address (RFC 2131 section 4.3.3).
const DECLINE: u8 = 4;

/// `DHCPACK`: the server confirms an address (RFC 2131 section 4.3.1).
const ACK: u8 = 5;

/// `DHCPRELEASE`: the client gives an address back (RFC 2131 section 4.3.5).
const RELEASE: u8 = 7;

/// `DHCPINFORM`: the client already has an address and wants only the other parameters
/// (RFC 2131 section 4.3.6).
const INFORM: u8 = 8;

/// The bytes after the magic cookie that every reply carries: option 53 (3 bytes), 54, 51, 1, 3,
/// 6 and 28 (6 bytes each) and the end marker (1 byte).
const REPLY_OPTIONS_LEN: usize = 3 + (6 * 6) + 1;

/// The length of every reply this server builds.
const REPLY_LEN: usize = HEADER_LEN + MAGIC_COOKIE.len() + REPLY_OPTIONS_LEN;

/// Half a `u32`'s range, used to compare timestamps across a wrap.
const HALF_RANGE: u32 = 1 << 31;

/// The pool's fixed configuration.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct ServerConfig {
    /// The server's own address, which is also the router, DNS server and server identifier.
    pub server: [u8; 4],
    /// The subnet prefix length; 32 or fewer.
    pub prefix: u8,
    /// The addresses the server may lease, in allocation order.
    pub pool: [[u8; 4]; POOL_SIZE],
}

impl ServerConfig {
    /// `192.168.4.1/24` with the pool `192.168.4.100` and `192.168.4.101`.
    #[must_use]
    pub const fn standard() -> Self {
        Self {
            server: [192, 168, 4, 1],
            prefix: 24,
            pool: [[192, 168, 4, 100], [192, 168, 4, 101]],
        }
    }
}

/// One address held by one client.
#[derive(Clone, Copy, Debug)]
struct Lease {
    /// The client's six-byte Ethernet address.
    client: [u8; 6],
    /// When the lease stops being live, in the caller's millisecond scale.
    until_ms: u32,
}

impl Lease {
    /// Returns whether the lease is still live at `now_ms`.
    const fn is_live(&self, now_ms: u32) -> bool {
        !reached(now_ms, self.until_ms)
    }
}

/// The parts of a client request this server acts on.
#[derive(Clone, Copy, Debug)]
struct Request {
    /// The transaction id, echoed into the reply.
    xid: [u8; 4],
    /// The BOOTP flags, echoed into the reply.
    flags: [u8; 2],
    /// The client's six-byte Ethernet address.
    chaddr: [u8; 6],
    /// The message type from option 53.
    message_type: u8,
    /// The address from option 50, when the client asked for one.
    requested: Option<[u8; 4]>,
}

/// The AP's DHCP server: the configuration and the two lease slots.
pub struct Server {
    /// The pool, server identity and prefix.
    config: ServerConfig,
    /// The two lease slots, in pool order.
    leases: [Option<Lease>; POOL_SIZE],
    /// The most recent clock reading, so [`Server::leases`] can ignore expired entries.
    now_ms: u32,
}

impl Server {
    /// Creates a server with an empty pool.
    #[must_use]
    pub const fn new(config: ServerConfig) -> Self {
        Self {
            config,
            leases: [None; POOL_SIZE],
            now_ms: 0,
        }
    }

    /// Handles one BOOTP/DHCP datagram, writing the reply into `out`.
    ///
    /// Returns the reply's length and kind, or `None` for malformed input, a message type the
    /// server does not answer, a release or decline, a full pool, or an `out` shorter than the
    /// reply.
    pub fn handle(&mut self, request: &[u8], now_ms: u32, out: &mut [u8]) -> Option<Reply> {
        self.now_ms = now_ms;
        if self.config.prefix > 32 || out.len() < REPLY_LEN {
            return None;
        }
        let request = parse(request)?;
        match request.message_type {
            DISCOVER => self.respond(&request, now_ms, false, OFFER, out).map(|length| Reply {
                length,
                kind: ReplyKind::Offer,
            }),
            REQUEST => self.respond(&request, now_ms, true, ACK, out).map(|length| Reply {
                length,
                kind: ReplyKind::Ack,
            }),
            DECLINE | RELEASE => {
                self.free(&request);
                None
            }
            INFORM => self.reply(&request, [0; 4], ACK, out).map(|length| Reply {
                length,
                kind: ReplyKind::Ack,
            }),
            _ => None,
        }
    }

    /// Returns how many leases were live at the most recent [`Server::handle`] clock reading.
    #[must_use]
    pub fn leases(&self) -> usize {
        self.leases
            .iter()
            .filter(|slot| slot.is_some_and(|lease| lease.is_live(self.now_ms)))
            .count()
    }

    /// Allocates a slot for a discover or request and builds the matching reply.
    ///
    /// A request may pin a free pool address through option 50; a discover always takes the first
    /// free or expired slot.
    fn respond(
        &mut self,
        request: &Request,
        now_ms: u32,
        honour_requested: bool,
        message_type: u8,
        out: &mut [u8],
    ) -> Option<usize> {
        let index = self.allocate(request, now_ms, honour_requested)?;
        let address = *self.config.pool.get(index)?;
        self.reply(request, address, message_type, out)
    }

    /// Picks a slot for the client, extending its live lease or taking the first free one.
    fn allocate(&mut self, request: &Request, now_ms: u32, honour_requested: bool) -> Option<usize> {
        if let Some(index) = self.live_index(request.chaddr, now_ms) {
            self.lease(index, request.chaddr, now_ms);
            return Some(index);
        }
        if honour_requested
            && let Some(requested) = request.requested
            && let Some(index) = self.pool_index(requested)
            && self.slot_free(index, now_ms)
        {
            self.lease(index, request.chaddr, now_ms);
            return Some(index);
        }
        let index = (0..POOL_SIZE).find(|index| self.slot_free(*index, now_ms))?;
        self.lease(index, request.chaddr, now_ms);
        Some(index)
    }

    /// Writes a lease for `client` into `index`.
    fn lease(&mut self, index: usize, client: [u8; 6], now_ms: u32) {
        if let Some(slot) = self.leases.get_mut(index) {
            *slot = Some(Lease {
                client,
                until_ms: now_ms.wrapping_add(LEASE_MS),
            });
        }
    }

    /// Drops the lease held by the request's client, if any.
    fn free(&mut self, request: &Request) {
        for slot in &mut self.leases {
            if slot.is_some_and(|lease| lease.client == request.chaddr) {
                *slot = None;
            }
        }
    }

    /// Returns the slot holding this client's live lease.
    fn live_index(&self, client: [u8; 6], now_ms: u32) -> Option<usize> {
        self.leases
            .iter()
            .position(|slot| slot.is_some_and(|lease| lease.client == client && lease.is_live(now_ms)))
    }

    /// Returns the pool slot for `address`, whether or not it is free.
    fn pool_index(&self, address: [u8; 4]) -> Option<usize> {
        self.config.pool.iter().position(|candidate| *candidate == address)
    }

    /// Returns whether `index` holds no lease or an expired one.
    fn slot_free(&self, index: usize, now_ms: u32) -> bool {
        match self.leases.get(index) {
            Some(Some(lease)) => !lease.is_live(now_ms),
            Some(None) => true,
            None => false,
        }
    }

    /// Builds one reply: the BOOTP header, the magic cookie and the fixed option set.
    // The lint counts the inlined option writers' temporaries as part of this frame; the function
    // itself borrows the caller's buffer and holds no large arrays.
    #[allow(clippy::large_stack_frames)]
    fn reply(&self, request: &Request, yiaddr: [u8; 4], message_type: u8, out: &mut [u8]) -> Option<usize> {
        if out.len() < REPLY_LEN {
            return None;
        }
        let mask = subnet_mask(self.config.prefix)?;
        let server = u32::from_be_bytes(self.config.server);
        let broadcast = (server | !mask).to_be_bytes();
        let mask = mask.to_be_bytes();

        out[..HEADER_LEN].fill(0);
        out[0] = OP_REPLY;
        out[1] = HARDWARE_ETHERNET;
        out[2] = HARDWARE_LEN;
        out[3] = 0;
        out[4..8].copy_from_slice(&request.xid);
        out[10..12].copy_from_slice(&request.flags);
        out[16..20].copy_from_slice(&yiaddr);
        out[20..24].copy_from_slice(&self.config.server);
        out[28..34].copy_from_slice(&request.chaddr);
        let mut index = HEADER_LEN;
        out[index..index + MAGIC_COOKIE.len()].copy_from_slice(&MAGIC_COOKIE);
        index += MAGIC_COOKIE.len();

        index = write_option(out, index, OPT_MESSAGE_TYPE, &[message_type])?;
        index = write_option(out, index, OPT_SERVER_ID, &self.config.server)?;
        index = write_option(out, index, OPT_LEASE_TIME, &LEASE_MS.to_be_bytes())?;
        index = write_option(out, index, OPT_SUBNET_MASK, &mask)?;
        index = write_option(out, index, OPT_ROUTER, &self.config.server)?;
        index = write_option(out, index, OPT_DNS, &self.config.server)?;
        index = write_option(out, index, OPT_BROADCAST, &broadcast)?;
        *out.get_mut(index)? = OPT_END;
        Some(index + 1)
    }
}

/// Parses the fixed header, the magic cookie and the options this server needs.
fn parse(datagram: &[u8]) -> Option<Request> {
    if datagram.len() < HEADER_LEN + MAGIC_COOKIE.len() {
        return None;
    }
    if datagram[0] != OP_REQUEST
        || datagram[1] != HARDWARE_ETHERNET
        || datagram[2] != HARDWARE_LEN
        || datagram[HEADER_LEN..HEADER_LEN + MAGIC_COOKIE.len()] != MAGIC_COOKIE
    {
        return None;
    }
    let mut chaddr = [0u8; 6];
    chaddr.copy_from_slice(datagram.get(28..34)?);
    let (message_type, requested) = parse_options(datagram, HEADER_LEN + MAGIC_COOKIE.len())?;
    Some(Request {
        xid: [datagram[4], datagram[5], datagram[6], datagram[7]],
        flags: [datagram[10], datagram[11]],
        chaddr,
        message_type,
        requested,
    })
}

/// Parses option 53 and option 50, skipping every other option.
///
/// Returns `None` when an option runs past the datagram or a known option has the wrong length.
fn parse_options(datagram: &[u8], start: usize) -> Option<(u8, Option<[u8; 4]>)> {
    let mut message_type = None;
    let mut requested = None;
    let mut index = start;
    while index < datagram.len() {
        let code = *datagram.get(index)?;
        index += 1;
        if code == OPT_END {
            break;
        }
        if code == OPT_PAD {
            continue;
        }
        let length = usize::from(*datagram.get(index)?);
        index += 1;
        let end = index.checked_add(length)?;
        if end > datagram.len() {
            return None;
        }
        match code {
            OPT_MESSAGE_TYPE => {
                if length != 1 {
                    return None;
                }
                message_type = Some(*datagram.get(index)?);
            }
            OPT_REQUESTED_IP => {
                if length != 4 {
                    return None;
                }
                let mut address = [0u8; 4];
                address.copy_from_slice(datagram.get(index..end)?);
                requested = Some(address);
            }
            _ => {}
        }
        index = end;
    }
    Some((message_type?, requested))
}

/// Writes one option and its value, returning the offset after it.
fn write_option(out: &mut [u8], index: usize, code: u8, value: &[u8]) -> Option<usize> {
    let length = u8::try_from(value.len()).ok()?;
    let end = index.checked_add(2)?.checked_add(value.len())?;
    if end > out.len() {
        return None;
    }
    out[index] = code;
    out[index + 1] = length;
    out[index + 2..end].copy_from_slice(value);
    Some(end)
}

/// Derives a big-endian-friendly subnet mask from a prefix length.
fn subnet_mask(prefix: u8) -> Option<u32> {
    if prefix > 32 {
        return None;
    }
    if prefix == 0 {
        return Some(0);
    }
    Some(u32::MAX << (32 - u32::from(prefix)))
}

/// Whether `now_ms` has reached `deadline_ms`, tolerating a `u32` wrap.
const fn reached(now_ms: u32, deadline_ms: u32) -> bool {
    now_ms.wrapping_sub(deadline_ms) < HALF_RANGE
}
