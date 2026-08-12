local udp_table = DissectorTable.get("udp.port")

local altos_proto = Proto("altos", "Altos Pointcloud")

local magic_field = ProtoField.string("altos.magic", "Magic")
local nsec_field = ProtoField.uint32("altos.nsec", "Nanoseconds")
local sec_field = ProtoField.uint64("altos.sec", "Seconds")
local frameid_field = ProtoField.uint32("altos.frame_id", "Frame ID")
local radarid_field = ProtoField.uint8("altos.radar_id", "Radar ID")
local mode_field = ProtoField.uint8("altos.mode", "Mode")
local dataid_field = ProtoField.uint16("altos.dataid", "Data ID")
local length_field = ProtoField.uint32("altos.length", "Length")
local offset_field = ProtoField.uint32("altos.offset", "Offset")

local range_field = ProtoField.float("altos.range", "Range (Meters)")
local doppler_field = ProtoField.float("altos.doppler", "Doppler (Meters/s)")
local azi_field = ProtoField.float("altos.azimuth", "Azimuth (Radians)")
local ele_field = ProtoField.float("altos.elevation", "Elevation (Radians)")
local snr_field = ProtoField.float("altos.snr", "SNR")

altos_proto.fields = {
	magic_field,
	nsec_field,
	sec_field,
	frameid_field,
	radarid_field,
	mode_field,
	dataid_field,
	length_field,
	offset_field,
	range_field,
	doppler_field,
	azi_field,
	ele_field,
	snr_field,
}
altos_proto.prefs.decode_points = Pref.bool("Decode points", false, "Decode points in the payload. Can be slow")

function altos_proto.dissector(buf, pkt, tree)
	local buf_len = buf:len()
	if buf_len < 32 then return end
	if buf(0,4):uint() ~= 0x416c746f then return end

	local subtree = tree:add(altos_proto, buf(), "Altos Pointcloud Data")

	subtree:add(magic_field, buf(0,4))
	subtree:add_le(nsec_field, buf(4,4))
	subtree:add_le(sec_field, buf(8,8))
	subtree:add_le(frameid_field, buf(16,4))
	subtree:add_le(radarid_field, buf(20,1))
	subtree:add_le(mode_field, buf(21,1))
	subtree:add_le(dataid_field, buf(22,2))
	subtree:add_le(length_field, buf(24,4))
	subtree:add_le(offset_field, buf(28,4))

	if altos_proto.prefs.decode_points then
		local point = 0;
		local offset = 32
		while offset < buf_len do
			if buf_len - offset < 20 then break end
			local point_buf = buf(offset,20)
			local point_subtree = subtree:add(altos_proto, point_buf(), "Point " .. point)
			point_subtree:add_le(range_field, point_buf(0,4))
			point_subtree:add_le(doppler_field, point_buf(4,4))
			point_subtree:add_le(azi_field, point_buf(8,4))
			point_subtree:add_le(ele_field, point_buf(12,4))
			point_subtree:add_le(snr_field, point_buf(16,4))
			offset = offset + 20
			point = point + 1
		end
	end
end

udp_table:add("4040-4049", altos_proto)
