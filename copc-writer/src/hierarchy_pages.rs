//! COPC hierarchy page planning and serialization.

use std::io::{Seek, Write};

use copc_core::{Entry, Error, Result, VoxelKey, HIERARCHY_ENTRY_BYTES};

const HIERARCHY_PAGE_MAX_ENTRIES: usize = 4_096;

#[derive(Debug)]
pub(crate) struct HierarchyPagePlan {
    pub(crate) key: VoxelKey,
    pub(crate) items: Vec<HierarchyPageItem>,
    pub(crate) offset: u64,
    pub(crate) byte_size: u64,
}

#[derive(Debug)]
pub(crate) enum HierarchyPageItem {
    Point(Entry),
    Child(Box<HierarchyPagePlan>),
}

pub(crate) fn plan_hierarchy_pages(entries: &[Entry], key: VoxelKey) -> Result<HierarchyPagePlan> {
    if entries.is_empty() {
        return Err(Error::InvalidInput(
            "cannot write empty hierarchy page".into(),
        ));
    }
    if entries.len() <= HIERARCHY_PAGE_MAX_ENTRIES {
        return Ok(HierarchyPagePlan {
            key,
            items: entries
                .iter()
                .copied()
                .map(HierarchyPageItem::Point)
                .collect(),
            offset: 0,
            byte_size: 0,
        });
    }

    let mut point_entry = None;
    let mut child_entries: [Vec<Entry>; 8] = std::array::from_fn(|_| Vec::new());
    for entry in entries.iter().copied() {
        if entry.key == key {
            point_entry = Some(entry);
            continue;
        }
        let mut matched = false;
        for (octant, child_entries) in child_entries.iter_mut().enumerate() {
            let child_key = key.child(octant as u8);
            if key_contains(child_key, entry.key) {
                child_entries.push(entry);
                matched = true;
                break;
            }
        }
        if !matched {
            return Err(Error::InvalidInput(format!(
                "hierarchy entry {:?} is not under page key {:?}",
                entry.key, key
            )));
        }
    }

    let mut items = Vec::new();
    if let Some(entry) = point_entry {
        items.push(HierarchyPageItem::Point(entry));
    }
    for (octant, child_entries) in child_entries.into_iter().enumerate() {
        if child_entries.is_empty() {
            continue;
        }
        items.push(HierarchyPageItem::Child(Box::new(plan_hierarchy_pages(
            &child_entries,
            key.child(octant as u8),
        )?)));
    }
    if items.len() > HIERARCHY_PAGE_MAX_ENTRIES {
        return Err(Error::InvalidInput(format!(
            "hierarchy page for {:?} has {} entries, max is {}",
            key,
            items.len(),
            HIERARCHY_PAGE_MAX_ENTRIES
        )));
    }
    Ok(HierarchyPagePlan {
        key,
        items,
        offset: 0,
        byte_size: 0,
    })
}

pub(crate) fn assign_hierarchy_page_offsets(
    page: &mut HierarchyPagePlan,
    offset: u64,
) -> Result<u64> {
    page.offset = offset;
    page.byte_size = hierarchy_page_byte_size(page.items.len())?;
    let mut next = offset
        .checked_add(page.byte_size)
        .ok_or_else(|| Error::InvalidInput("hierarchy page offset overflow".into()))?;
    for item in &mut page.items {
        if let HierarchyPageItem::Child(child) = item {
            next = assign_hierarchy_page_offsets(child, next)?;
        }
    }
    Ok(next)
}

fn hierarchy_page_byte_size(entry_count: usize) -> Result<u64> {
    let bytes = entry_count
        .checked_mul(HIERARCHY_ENTRY_BYTES)
        .ok_or_else(|| Error::InvalidInput("hierarchy page size overflow".into()))?;
    u64::try_from(bytes).map_err(|_| Error::InvalidInput("hierarchy page is too large".into()))
}

pub(crate) fn write_hierarchy_page_tree<W: Write + Seek>(
    writer: &mut W,
    page: &HierarchyPagePlan,
) -> Result<()> {
    let position = writer
        .stream_position()
        .map_err(|e| Error::io("record hierarchy page offset", e))?;
    if position != page.offset {
        return Err(Error::InvalidInput(format!(
            "hierarchy page offset mismatch: at {position}, expected {}",
            page.offset
        )));
    }
    let mut entry_buf = [0u8; HIERARCHY_ENTRY_BYTES];
    for item in &page.items {
        hierarchy_page_item_entry(item)?.write_le(&mut entry_buf)?;
        writer
            .write_all(&entry_buf)
            .map_err(|e| Error::io("write hierarchy entry", e))?;
    }
    for item in &page.items {
        if let HierarchyPageItem::Child(child) = item {
            write_hierarchy_page_tree(writer, child)?;
        }
    }
    Ok(())
}

fn hierarchy_page_item_entry(item: &HierarchyPageItem) -> Result<Entry> {
    match item {
        HierarchyPageItem::Point(entry) => Ok(*entry),
        HierarchyPageItem::Child(child) => Ok(Entry {
            key: child.key,
            offset: child.offset,
            byte_size: i32::try_from(child.byte_size).map_err(|_| {
                Error::InvalidInput("child hierarchy page exceeds COPC i32 byte size".into())
            })?,
            point_count: -1,
        }),
    }
}

fn key_contains(ancestor: VoxelKey, key: VoxelKey) -> bool {
    if key.level < ancestor.level {
        return false;
    }
    let shift = (key.level - ancestor.level) as u32;
    (key.x >> shift) == ancestor.x
        && (key.y >> shift) == ancestor.y
        && (key.z >> shift) == ancestor.z
}

#[cfg(test)]
mod tests {
    use super::*;

    use std::io::{Cursor, SeekFrom};

    #[test]
    fn hierarchy_plan_splits_large_root_page() {
        let mut entries = vec![Entry {
            key: VoxelKey::root(),
            offset: 1,
            byte_size: 1,
            point_count: 1,
        }];
        let mut offset = 2;
        for z in 0..16 {
            for y in 0..16 {
                for x in 0..16 {
                    entries.push(Entry {
                        key: VoxelKey { level: 4, x, y, z },
                        offset,
                        byte_size: 1,
                        point_count: 1,
                    });
                    offset += 1;
                }
            }
        }
        entries.sort_by_key(|entry| entry.key);

        let mut plan = plan_hierarchy_pages(&entries, VoxelKey::root()).unwrap();
        let start = 1024;
        let end = assign_hierarchy_page_offsets(&mut plan, start).unwrap();

        assert!(plan.byte_size < hierarchy_page_byte_size(entries.len()).unwrap());
        assert!(plan
            .items
            .iter()
            .any(|item| matches!(item, HierarchyPageItem::Child(_))));

        let mut out = Cursor::new(vec![0; start as usize]);
        out.seek(SeekFrom::Start(start)).unwrap();
        write_hierarchy_page_tree(&mut out, &plan).unwrap();
        assert_eq!(end, out.get_ref().len() as u64);
    }
}
