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

    pub fn child(self, octant: u8) -> Result<Self> {
        self.validate()?;
        if octant > 7 {
            return Err(Error::InvalidInput(format!(
                "octant must be in 0..=7, got {octant}"
            )));
        }
        let level = self
            .level
            .checked_add(1)
            .ok_or_else(|| Error::InvalidInput("voxel level overflow".into()))?;
        let child_axis = |axis: i32, bit: u8| {
            axis.checked_mul(2)
                .and_then(|axis| axis.checked_add(i32::from(bit)))
                .ok_or_else(|| Error::InvalidInput("voxel axis overflow".into()))
        };
        let child = Self {
            level,
            x: child_axis(self.x, octant & 1)?,
            y: child_axis(self.y, (octant >> 1) & 1)?,
            z: child_axis(self.z, (octant >> 2) & 1)?,
        };
        child.validate()?;
        Ok(child)
    }

    /// Validate this key against the EPT/COPC octree coordinate rules.
    pub fn validate(self) -> Result<()> {
        if self.level < 0 || self.x < 0 || self.y < 0 || self.z < 0 {
            return Err(Error::InvalidData(format!(
                "invalid negative COPC voxel key {self:?}"
            )));
        }
        if self.level < 31 {
            let axis_limit = 1i32 << self.level;
            if self.x >= axis_limit || self.y >= axis_limit || self.z >= axis_limit {
                return Err(Error::InvalidData(format!(
                    "COPC voxel key {self:?} has an axis outside 0..{axis_limit}"
                )));
            }
        }
        Ok(())
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

/// Availability represented by a COPC hierarchy entry.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum EntryAvailability {
    Empty,
    PointData { point_count: u32 },
    ChildPage,
}

impl Entry {
    /// Validate invariants that do not depend on the containing file.
    pub fn validate(self) -> Result<()> {
        self.key.validate()?;
        match self.availability()? {
            EntryAvailability::Empty => {
                if self.offset != 0 || self.byte_size != 0 {
                    return Err(Error::InvalidData(format!(
                        "empty hierarchy entry {:?} must have zero offset and byte size",
                        self.key
                    )));
                }
            }
            EntryAvailability::PointData { .. } => {
                if self.byte_size <= 0 {
                    return Err(Error::InvalidData(format!(
                        "point data entry {:?} has invalid byte size {}",
                        self.key, self.byte_size
                    )));
                }
            }
            EntryAvailability::ChildPage => {
                if self.byte_size <= 0 {
                    return Err(Error::InvalidData(format!(
                        "child hierarchy page {:?} has invalid byte size {}",
                        self.key, self.byte_size
                    )));
                }
            }
        }
        Ok(())
    }

    pub fn availability(self) -> Result<EntryAvailability> {
        match self.point_count {
            -1 => Ok(EntryAvailability::ChildPage),
            0 => Ok(EntryAvailability::Empty),
            count if count > 0 => {
                let point_count = u32::try_from(count).map_err(|_| {
                    Error::InvalidData(format!(
                        "hierarchy entry {:?} point count {} is out of range",
                        self.key, self.point_count
                    ))
                })?;
                Ok(EntryAvailability::PointData { point_count })
            }
            _ => Err(Error::InvalidData(format!(
                "hierarchy entry {:?} has invalid point count {}",
                self.key, self.point_count
            ))),
        }
    }

    pub fn has_point_data(self) -> bool {
        self.point_count > 0
    }

    pub fn is_empty(self) -> bool {
        self.point_count == 0
    }

    pub fn is_child_page(self) -> bool {
        self.point_count == -1
    }

    pub fn write_le(self, dst: &mut [u8]) -> Result<()> {
        self.validate()?;
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
        let entry = Self {
            key: VoxelKey {
                level: i32::from_le_bytes(src[0..4].try_into().expect("level width")),
                x: i32::from_le_bytes(src[4..8].try_into().expect("x width")),
                y: i32::from_le_bytes(src[8..12].try_into().expect("y width")),
                z: i32::from_le_bytes(src[12..16].try_into().expect("z width")),
            },
            offset: u64::from_le_bytes(src[16..24].try_into().expect("offset width")),
            byte_size: i32::from_le_bytes(src[24..28].try_into().expect("byte_size width")),
            point_count: i32::from_le_bytes(src[28..32].try_into().expect("point_count width")),
        };
        entry.validate()?;
        Ok(entry)
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
        let byte_len = self
            .entries
            .len()
            .checked_mul(HIERARCHY_ENTRY_BYTES)
            .ok_or_else(|| Error::InvalidInput("hierarchy page size overflows usize".into()))?;
        let mut out = vec![0u8; byte_len];
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
        let child = VoxelKey::root().child(0b101).unwrap();
        assert_eq!(
            child,
            VoxelKey {
                level: 1,
                x: 1,
                y: 0,
                z: 1,
            }
        );
        assert!(VoxelKey::root().child(8).is_err());
    }

    #[test]
    fn voxel_key_validation_rejects_invalid_axes() {
        VoxelKey::root().validate().unwrap();
        VoxelKey {
            level: 3,
            x: 7,
            y: 7,
            z: 7,
        }
        .validate()
        .unwrap();
        assert!(VoxelKey {
            level: 3,
            x: 8,
            y: 0,
            z: 0,
        }
        .validate()
        .is_err());
        assert!(VoxelKey {
            level: -1,
            x: 0,
            y: 0,
            z: 0,
        }
        .validate()
        .is_err());
    }

    #[test]
    fn entry_availability_classifies_point_count() {
        let key = VoxelKey::root();
        assert_eq!(
            Entry {
                key,
                offset: 0,
                byte_size: 0,
                point_count: 0
            }
            .availability()
            .unwrap(),
            EntryAvailability::Empty
        );
        assert_eq!(
            Entry {
                key,
                offset: 64,
                byte_size: 128,
                point_count: 42
            }
            .availability()
            .unwrap(),
            EntryAvailability::PointData { point_count: 42 }
        );
        assert_eq!(
            Entry {
                key,
                offset: 64,
                byte_size: 128,
                point_count: -1
            }
            .availability()
            .unwrap(),
            EntryAvailability::ChildPage
        );
    }
}
