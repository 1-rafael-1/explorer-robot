//! Autonomous drive mode tasks.
//!
//! [`coast_obstacle_avoid`] is the live mode: the UI's `LiDAR` warm-up helper
//! spawns its task with the one Active Claim.
//!
//! [`attempt_straight_line`] and [`gap_analysis`] are deferred and unwired. Nothing
//! starts the attempt-straight-line task, so its private `ACTIVE` flag is dead code
//! rather than an authority; when the mode is revived it must take the Active Claim
//! from [`crate::task::procedure`] instead of keeping a flag. There is no
//! autonomous-mode controller any more.
//!
//! Gap analysis uses the `lidar-cloud` cloud and obstacle detection reads the
//! lock-free flag the `LiDAR` task drives through that crate's Front Sector test.

pub mod attempt_straight_line;
pub mod coast_obstacle_avoid;
pub mod gap_analysis;
