//! LAS 1.4 header, VLR, and EVLR serialization for COPC output.

use std::io::Write;

use byteorder::{LittleEndian, WriteBytesExt};
use copc_core::{Bounds, Error, Result};

pub(crate) const LAS_VLR_HEADER_BYTES: u32 = 54;
pub(crate) const LAS_EVLR_HEADER_BYTES: u64 = 60;

pub(crate) struct LasHeader {
    pub(crate) point_data_format: u8,
    pub(crate) point_record_length: u16,
    pub(crate) offset_to_point_data: u32,
    pub(crate) number_of_vlrs: u32,
    pub(crate) file_source_id: u16,
    pub(crate) global_encoding: u16,
    pub(crate) guid: [u8; 16],
    pub(crate) system_identifier: String,
    pub(crate) generating_software: String,
    pub(crate) creation_day_of_year: u16,
    pub(crate) creation_year: u16,
    pub(crate) scale: (f64, f64, f64),
    pub(crate) offset: (f64, f64, f64),
    pub(crate) bounds: Bounds,
    pub(crate) legacy_point_count: u32,
    pub(crate) total_point_count: u64,
    pub(crate) offset_to_first_evlr: u64,
    pub(crate) number_of_evlrs: u32,
    pub(crate) extended_return_counts: [u64; 15],
}

impl LasHeader {
    pub(crate) fn write<W: Write>(&self, writer: &mut W) -> Result<()> {
        writer
            .write_all(b"LASF")
            .map_err(|e| Error::io("write LAS signature", e))?;
        writer
            .write_u16::<LittleEndian>(self.file_source_id)
            .map_err(|e| Error::io("write file source id", e))?;
        writer
            .write_u16::<LittleEndian>(self.global_encoding)
            .map_err(|e| Error::io("write global encoding", e))?;
        writer
            .write_all(&self.guid)
            .map_err(|e| Error::io("write GUID", e))?;
        writer
            .write_u8(1)
            .map_err(|e| Error::io("write version major", e))?;
        writer
            .write_u8(4)
            .map_err(|e| Error::io("write version minor", e))?;
        writer
            .write_all(&pad(self.system_identifier.as_bytes(), 32))
            .map_err(|e| Error::io("write system id", e))?;
        writer
            .write_all(&pad(self.generating_software.as_bytes(), 32))
            .map_err(|e| Error::io("write generating software", e))?;
        writer
            .write_u16::<LittleEndian>(self.creation_day_of_year)
            .map_err(|e| Error::io("write creation day", e))?;
        writer
            .write_u16::<LittleEndian>(self.creation_year)
            .map_err(|e| Error::io("write creation year", e))?;
        writer
            .write_u16::<LittleEndian>(375)
            .map_err(|e| Error::io("write header size", e))?;
        writer
            .write_u32::<LittleEndian>(self.offset_to_point_data)
            .map_err(|e| Error::io("write point data offset", e))?;
        writer
            .write_u32::<LittleEndian>(self.number_of_vlrs)
            .map_err(|e| Error::io("write VLR count", e))?;
        writer
            .write_u8(self.point_data_format)
            .map_err(|e| Error::io("write point format", e))?;
        writer
            .write_u16::<LittleEndian>(self.point_record_length)
            .map_err(|e| Error::io("write point record length", e))?;
        writer
            .write_u32::<LittleEndian>(self.legacy_point_count)
            .map_err(|e| Error::io("write legacy point count", e))?;
        for _ in 0..5 {
            writer
                .write_u32::<LittleEndian>(0)
                .map_err(|e| Error::io("write legacy returns", e))?;
        }
        writer
            .write_f64::<LittleEndian>(self.scale.0)
            .map_err(|e| Error::io("write x scale", e))?;
        writer
            .write_f64::<LittleEndian>(self.scale.1)
            .map_err(|e| Error::io("write y scale", e))?;
        writer
            .write_f64::<LittleEndian>(self.scale.2)
            .map_err(|e| Error::io("write z scale", e))?;
        writer
            .write_f64::<LittleEndian>(self.offset.0)
            .map_err(|e| Error::io("write x offset", e))?;
        writer
            .write_f64::<LittleEndian>(self.offset.1)
            .map_err(|e| Error::io("write y offset", e))?;
        writer
            .write_f64::<LittleEndian>(self.offset.2)
            .map_err(|e| Error::io("write z offset", e))?;
        writer
            .write_f64::<LittleEndian>(self.bounds.max.0)
            .map_err(|e| Error::io("write max x", e))?;
        writer
            .write_f64::<LittleEndian>(self.bounds.min.0)
            .map_err(|e| Error::io("write min x", e))?;
        writer
            .write_f64::<LittleEndian>(self.bounds.max.1)
            .map_err(|e| Error::io("write max y", e))?;
        writer
            .write_f64::<LittleEndian>(self.bounds.min.1)
            .map_err(|e| Error::io("write min y", e))?;
        writer
            .write_f64::<LittleEndian>(self.bounds.max.2)
            .map_err(|e| Error::io("write max z", e))?;
        writer
            .write_f64::<LittleEndian>(self.bounds.min.2)
            .map_err(|e| Error::io("write min z", e))?;
        writer
            .write_u64::<LittleEndian>(0)
            .map_err(|e| Error::io("write waveform packet start", e))?;
        writer
            .write_u64::<LittleEndian>(self.offset_to_first_evlr)
            .map_err(|e| Error::io("write first EVLR offset", e))?;
        writer
            .write_u32::<LittleEndian>(self.number_of_evlrs)
            .map_err(|e| Error::io("write EVLR count", e))?;
        writer
            .write_u64::<LittleEndian>(self.total_point_count)
            .map_err(|e| Error::io("write total point count", e))?;
        for count in self.extended_return_counts {
            writer
                .write_u64::<LittleEndian>(count)
                .map_err(|e| Error::io("write extended returns", e))?;
        }
        Ok(())
    }
}

fn pad(value: &[u8], len: usize) -> Vec<u8> {
    let mut out = Vec::with_capacity(len);
    let take = value.len().min(len);
    out.extend_from_slice(&value[..take]);
    out.resize(len, 0);
    out
}

pub(crate) fn write_vlr_header<W: Write>(
    writer: &mut W,
    user_id: &str,
    record_id: u16,
    body_size: u16,
    description: &str,
) -> Result<()> {
    writer
        .write_u16::<LittleEndian>(0)
        .map_err(|e| Error::io("write VLR reserved", e))?;
    writer
        .write_all(&pad(user_id.as_bytes(), 16))
        .map_err(|e| Error::io("write VLR user id", e))?;
    writer
        .write_u16::<LittleEndian>(record_id)
        .map_err(|e| Error::io("write VLR record id", e))?;
    writer
        .write_u16::<LittleEndian>(body_size)
        .map_err(|e| Error::io("write VLR body size", e))?;
    writer
        .write_all(&pad(description.as_bytes(), 32))
        .map_err(|e| Error::io("write VLR description", e))?;
    Ok(())
}

pub(crate) fn write_las_vlr<W: Write>(writer: &mut W, vlr: &las::Vlr) -> Result<()> {
    let body_size = u16::try_from(vlr.data.len()).map_err(|_| {
        Error::InvalidInput(format!(
            "regular VLR {}:{} is too large: {} byte(s)",
            vlr.user_id,
            vlr.record_id,
            vlr.data.len()
        ))
    })?;
    write_vlr_header(
        writer,
        &vlr.user_id,
        vlr.record_id,
        body_size,
        &vlr.description,
    )?;
    writer
        .write_all(&vlr.data)
        .map_err(|e| Error::io("write VLR body", e))?;
    Ok(())
}

pub(crate) fn regular_las_vlrs_bytes(vlrs: &[las::Vlr]) -> Result<u32> {
    vlrs.iter().try_fold(0u32, |total, vlr| {
        let data_len = u16::try_from(vlr.data.len()).map_err(|_| {
            Error::InvalidInput(format!(
                "regular VLR {}:{} is too large: {} byte(s)",
                vlr.user_id,
                vlr.record_id,
                vlr.data.len()
            ))
        })?;
        total
            .checked_add(LAS_VLR_HEADER_BYTES + u32::from(data_len))
            .ok_or_else(|| Error::InvalidInput("VLR byte size overflow".into()))
    })
}

pub(crate) fn write_evlr_header<W: Write>(
    writer: &mut W,
    user_id: &str,
    record_id: u16,
    body_size: u64,
    description: &str,
) -> Result<()> {
    writer
        .write_u16::<LittleEndian>(0)
        .map_err(|e| Error::io("write EVLR reserved", e))?;
    writer
        .write_all(&pad(user_id.as_bytes(), 16))
        .map_err(|e| Error::io("write EVLR user id", e))?;
    writer
        .write_u16::<LittleEndian>(record_id)
        .map_err(|e| Error::io("write EVLR record id", e))?;
    writer
        .write_u64::<LittleEndian>(body_size)
        .map_err(|e| Error::io("write EVLR body size", e))?;
    writer
        .write_all(&pad(description.as_bytes(), 32))
        .map_err(|e| Error::io("write EVLR description", e))?;
    Ok(())
}

pub(crate) fn write_las_evlr<W: Write>(writer: &mut W, vlr: &las::Vlr) -> Result<()> {
    let body_size = u64::try_from(vlr.data.len()).map_err(|_| {
        Error::InvalidInput(format!(
            "EVLR {}:{} is too large: {} byte(s)",
            vlr.user_id,
            vlr.record_id,
            vlr.data.len()
        ))
    })?;
    write_evlr_header(
        writer,
        &vlr.user_id,
        vlr.record_id,
        body_size,
        &vlr.description,
    )?;
    writer
        .write_all(&vlr.data)
        .map_err(|e| Error::io("write EVLR body", e))?;
    Ok(())
}
