// Copyright © 2026 Cyberus Technology GmbH
//
// SPDX-License-Identifier: Apache-2.0

use crate::ramfb::RamfbConfig;

/// A framebuffer whose pixels can be snapshotted by a display backend.
///
/// Implementations must return `None` while the framebuffer is not mapped.
/// This lets consumers stop touching a DMA mapping before it is removed.
pub trait FramebufferSource: Send + Sync {
    /// Return the active framebuffer configuration, if one is available.
    fn config(&self) -> Option<RamfbConfig>;

    /// Copy the complete framebuffer into an owned snapshot.
    fn read_framebuffer(&self) -> Option<Vec<u8>>;

    fn is_initialized(&self) -> bool {
        self.config().is_some()
    }
}
