use crate::{Error, Result};

pub const HIERARCHY_ENTRY_BYTES: usize = 32;

/// COPC octree voxel key.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash, PartialOrd, Ord)]
pub struct VoxelKey {
    pub level: i32,
    pub x: i32,
    pub y: i32,
    pub z: i32,
}

impl VoxelKey {
    pub const fn root() -> Self {
        Self {
            level: 0,
            x: 0,
            y: 0,
            z: 0,
        }
    }

    pub fn child(self, octant: u8) -> Self {
        Self {
            level: self.level + 1,
            x: (self.x << 1) | i32::from(octant & 1),
            y: (self.y << 1) | i32::from((octant >> 1) & 1),
            z: (self.z << 1) | i32::from((octant >> 2) & 1),
        }
    }
}

/// One 32-byte COPC hierarchy entry.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Entry {
    pub key: VoxelKey,
    pub offset: u64,
    pub byte_size: i32,
    pub point_count: i32,
}

impl Entry {
    pub fn is_child_page(self) -> bool {
        self.point_count == -1
    }

    pub fn write_le(self, dst: &mut [u8]) -> Result<()> {
        if dst.len() != HIERARCHY_ENTRY_BYTES {
            return Err(Error::InvalidInput(format!(
                "hierarchy entry destination is {} bytes, expected {}",
                dst.len(),
                HIERARCHY_ENTRY_BYTES
            )));
        }
        dst[0..4].copy_from_slice(&self.key.level.to_le_bytes());
        dst[4..8].copy_from_slice(&self.key.x.to_le_bytes());
        dst[8..12].copy_from_slice(&self.key.y.to_le_bytes());
        dst[12..16].copy_from_slice(&self.key.z.to_le_bytes());
        dst[16..24].copy_from_slice(&self.offset.to_le_bytes());
        dst[24..28].copy_from_slice(&self.byte_size.to_le_bytes());
        dst[28..32].copy_from_slice(&self.point_count.to_le_bytes());
        Ok(())
    }

    pub fn from_le(src: &[u8]) -> Result<Self> {
        if src.len() != HIERARCHY_ENTRY_BYTES {
            return Err(Error::InvalidData(format!(
                "hierarchy entry is {} bytes, expected {}",
                src.len(),
                HIERARCHY_ENTRY_BYTES
            )));
        }
        Ok(Self {
            key: VoxelKey {
                level: i32::from_le_bytes(src[0..4].try_into().expect("level width")),
                x: i32::from_le_bytes(src[4..8].try_into().expect("x width")),
                y: i32::from_le_bytes(src[8..12].try_into().expect("y width")),
                z: i32::from_le_bytes(src[12..16].try_into().expect("z width")),
            },
            offset: u64::from_le_bytes(src[16..24].try_into().expect("offset width")),
            byte_size: i32::from_le_bytes(src[24..28].try_into().expect("byte_size width")),
            point_count: i32::from_le_bytes(src[28..32].try_into().expect("point_count width")),
        })
    }
}

/// A COPC hierarchy page.
#[derive(Clone, Debug, Default, PartialEq, Eq)]
pub struct HierarchyPage {
    entries: Vec<Entry>,
}

impl HierarchyPage {
    pub fn new(entries: Vec<Entry>) -> Self {
        Self { entries }
    }

    pub fn entries(&self) -> &[Entry] {
        &self.entries
    }

    pub fn into_entries(self) -> Vec<Entry> {
        self.entries
    }

    pub fn from_le_bytes(bytes: &[u8]) -> Result<Self> {
        if bytes.len() % HIERARCHY_ENTRY_BYTES != 0 {
            return Err(Error::InvalidData(format!(
                "hierarchy page is {} bytes, not a multiple of {}",
                bytes.len(),
                HIERARCHY_ENTRY_BYTES
            )));
        }
        let entries = bytes
            .chunks_exact(HIERARCHY_ENTRY_BYTES)
            .map(Entry::from_le)
            .collect::<Result<Vec<_>>>()?;
        Ok(Self { entries })
    }

    pub fn write_le_bytes(&self) -> Result<Vec<u8>> {
        let mut out = vec![0u8; self.entries.len() * HIERARCHY_ENTRY_BYTES];
        for (entry, chunk) in self
            .entries
            .iter()
            .copied()
            .zip(out.chunks_exact_mut(HIERARCHY_ENTRY_BYTES))
        {
            entry.write_le(chunk)?;
        }
        Ok(out)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn hierarchy_entry_round_trips() {
        let entry = Entry {
            key: VoxelKey {
                level: 3,
                x: 4,
                y: 5,
                z: 6,
            },
            offset: 123_456,
            byte_size: 789,
            point_count: 42,
        };
        let mut bytes = [0u8; HIERARCHY_ENTRY_BYTES];
        entry.write_le(&mut bytes).unwrap();
        assert_eq!(Entry::from_le(&bytes).unwrap(), entry);
    }

    #[test]
    fn voxel_child_maps_octant_bits() {
        let child = VoxelKey::root().child(0b101);
        assert_eq!(
            child,
            VoxelKey {
                level: 1,
                x: 1,
                y: 0,
                z: 1,
            }
        );
    }
}
