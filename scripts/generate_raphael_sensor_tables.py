#!/usr/bin/env python3
"""Extract Raphael camera register tables from local CamX module binaries.

This reads data only. It never loads or executes vendor code.
The QTI format stores the first register value in the last 80-byte
RegSetting record; the remaining values follow in the registerData block.
OV8856's 3264x2448 geometry and stream-on register cross-check that layout.

Source: Evolution-X-Devices/vendor_xiaomi_raphael, commit
7b2e8518294b8a40452163b8ea7c04aa271ce82c, proprietary/vendor/lib64/camera/.
Supply the seven local sensormodule BIN files with --vendor-dir. Their SHA256
digests are pinned below; the files are not part of this kernel repository.
"""
import argparse
from pathlib import Path
import hashlib
import struct

ROOT = None
DEST = None
SOURCES = {
    "imx586": "raphael_luxvisions_imx586",
    "ov8856": "raphael_luxvisions_ov8856",
    "s5k3l6": "raphael_luxvisions_s5k3l6",
    "s5k3t2": "raphael_sunny_s5k3t2",
}
ALTERNATES = {
    "imx586": "raphael_ofilm_imx586",
    "ov8856": "raphael_ofilm_ov8856",
    "s5k3l6": "raphael_ofilm_s5k3l6",
}
EXPECTED_SHA256 = {
    "raphael_luxvisions_imx586": "0bf3dc55b096fcd9899361308f4581043260fa483c78eecd5353fe504de8d655",
    "raphael_luxvisions_ov8856": "b5b32a43310edd7f9b28470109572bb85f05d32a6af5dd7a556a54089981853e",
    "raphael_luxvisions_s5k3l6": "0421c932195f33e45a71bf9d38a4364c4f41b73ba8922b6f0f0f28992bdf2a40",
    "raphael_ofilm_imx586": "9d4122804022d2469d9162a486995bce877368f9c39105f6b7e063269fb3018f",
    "raphael_ofilm_ov8856": "9d61d03c0f2f40ba3019920fd7bdea93ea74fb4c1e09078c3177c075bf1dd96e",
    "raphael_ofilm_s5k3l6": "cb061ae446b283d5008e77c3c671fe8fc2051075a5ef1fcd81190c71d81c6b0d",
    "raphael_sunny_s5k3t2": "14eaf53535c8bac2a1e6ff50c900cc0297af6f90591a6e59508b77e63021e1d0",
}


def read_module(tag):
    path = ROOT / f"com.qti.sensormodule.{tag}.bin"
    payload = path.read_bytes()
    digest = hashlib.sha256(payload).hexdigest()
    if digest != EXPECTED_SHA256[tag]:
        raise ValueError(f"Unexpected SHA256 for {path}: {digest}")
    metadata_start = struct.unpack_from("<Q", payload, 0xa8)[0]
    metadata_bytes = struct.unpack_from("<Q", payload, 0xb0)[0]
    base = struct.unpack_from("<Q", payload, 0xc0)[0] + 16
    assert metadata_start == 0xe8 and metadata_bytes % 72 == 0
    fields = []
    for index in range(metadata_bytes // 72):
        off = metadata_start + index * 72
        name = payload[off + 8:off + 48].split(bytes([0]), 1)[0].decode()
        rel, size = struct.unpack_from("<QQ", payload, off + 56)
        fields.append((name, rel, size))
    return path, payload, base, fields


def extract_sequence(payload, base, fields, index):
    _, rel, size = fields[index]
    assert size and size % 80 == 0
    count = size // 80
    records = [struct.unpack_from("<10Q", payload, base + rel + j * 80)
               for j in range(count)]
    for j in range(count):
        assert [fields[index + 1 + 3*j + k][0] for k in range(3)] == [
            "slaveAddr", "registerData", "delayUs"]
    # The tail of the last record carries the first register value.
    values = [records[-1][8]]
    values += [struct.unpack_from("<Q", payload,
               base + fields[index + 2 + 3*j][1])[0]
               for j in range(count - 1)]
    # The final delayUs metadata slot is reused by the next field ID.
    delays = [struct.unpack_from("<Q", payload,
              base + fields[index + 3 + 3*j][1])[0]
              for j in range(count - 1)] + [0]
    result = []
    for row, value, delay in zip(records, values, delays):
        addr, addr_bytes, data_bytes, op = row[0], row[3], row[4], row[5]
        assert addr <= 0xffff and addr_bytes == 2 and data_bytes in (1, 2)
        assert op == 0 and value < 1 << (data_bytes * 8)
        assert delay <= 1000000
        result.append((addr, value, data_bytes, delay))
    return tuple(result)


def extract_sensor(tag):
    path, blob, base, fields = read_module(tag)
    sensor_rel = next(rel for name, rel, _ in fields if name == "sensorDriverData")
    slave = struct.unpack_from("<101Q", blob, base + sensor_rel)
    assert slave[4:6] == (2, 2) and slave[8] == 0xffffffff
    mode_meta = next((rel, size) for name, rel, size in fields
                     if name == "resolutionData")
    assert mode_meta[1] % 272 == 0
    count = mode_meta[1] // 272
    info_rel, info_size = next((rel, size) for name, rel, size in fields
                               if name == "resolutionInfo")
    assert info_size == 32
    info = struct.unpack_from("<4Q", blob, base + info_rel)
    assert info[0] == count and info[3] in (0, 1, 2, 3)
    stream_meta = [(rel, size) for name, rel, size in fields
                   if name == "streamConfiguration"]
    min_line_index = next(i for i, field in enumerate(fields)
                          if field[0] == "minLineCount")
    mode_indices = [i for i, field in enumerate(fields[:min_line_index])
                    if field[0] == "regSetting"]
    assert len(mode_indices) == count == len(stream_meta)
    modes = []
    for i, index in enumerate(mode_indices):
        timing = struct.unpack_from("<34Q", blob, base + mode_meta[0] + i * 272)
        stream = struct.unpack_from("<8Q", blob, base + stream_meta[i][0])
        width, height, bits = stream[2:5]
        output_pixel_clock, lanes = timing[5], timing[9]
        assert bits == 10 and lanes == 4
        assert output_pixel_clock * bits % (2 * lanes) == 0
        fps_x100 = round(struct.unpack("<d", struct.pack("<Q", timing[8]))[0] * 100)
        # OV8856 register HTS counts two sensor pixel clocks per unit.
        # The mainline OV8856 driver uses SCLK=144 MHz and converts its
        # 360 MHz / 4-lane RAW10 rate to 288 MHz, giving exactly 2× HTS.
        hts = timing[1] * (2 if "ov8856" in tag else 1)
        assert hts >= width and timing[2] >= height and fps_x100 > 0
        assert hts * timing[2] * fps_x100 // 100 <= 0xffffffff
        regs = extract_sequence(blob, base, fields, index)
        values = {address: value for address, value, _, _ in regs}
        if "imx586" in tag or "ov8856" in tag:
            x, y = ((0x3808, 0x380a) if "ov8856" in tag
                    else (0x034c, 0x034e))
            reg_width = values[x] * 256 + values[x + 1]
            reg_height = values[y] * 256 + values[y + 1]
        else:
            reg_width, reg_height = values[0x034c], values[0x034e]
        assert (reg_width, reg_height) == (width, height)
        if "imx586" in tag or "ov8856" in tag:
            line, frame = ((0x380c, 0x380e) if "ov8856" in tag
                           else (0x0342, 0x0340))
            reg_line = values[line] * 256 + values[line + 1]
            reg_frame = values[frame] * 256 + values[frame + 1]
        else:
            reg_line, reg_frame = values[0x0342], values[0x0340]
        assert (reg_line, reg_frame) == (timing[1], timing[2])
        modes.append({
            "source_mode": i,
            "width": width, "height": height,
            "hts": hts, "vts": timing[2],
            # V4L2 pixel rate is the internal VT clock, distinct from CSI output.
            "pixel_rate": (hts * timing[2] * fps_x100 + 50) // 100,
            "output_pixel_clock": output_pixel_clock,
            "link_freq": output_pixel_clock * bits // (2 * lanes),
            "fps_x100": fps_x100,
            "regs": regs,
        })
    init_index = next(i for i, field in enumerate(fields)
                      if field[0] == "initSettings")
    assert fields[init_index + 1][0] == "regSetting"
    stream_indices = [i for i, field in enumerate(fields[min_line_index:init_index],
                     min_line_index) if field[0] == "regSetting"]
    assert len(stream_indices) >= 4
    seq = [extract_sequence(blob, base, fields, i)
           for i in stream_indices[:4]]
    init = extract_sequence(blob, base, fields, init_index + 1)
    power_fields = [(rel, size) for name, rel, size in fields
                    if name == "powerSetting"][:2]
    assert len(power_fields) == 2
    power_sequences = []
    for rel, size in power_fields:
        assert size % 24 == 0
        power_sequences.append(tuple(struct.unpack_from("<3Q", blob,
                               base + rel + j * 24)
                               for j in range(size // 24)))
    max_analog_gain = struct.unpack("<d", struct.pack("<Q", slave[40]))[0]
    return {
        "path": path, "sha256": hashlib.sha256(blob).hexdigest(),
        "i2c_8bit": slave[3], "chip_id_register": slave[6], "chip_id": slave[7],
        "frame_length_register": slave[16],
        "exposure_register": slave[18],
        "analogue_gain_register": slave[21],
        "max_analog_gain": max_analog_gain,
        "vertical_offset": slave[44],
        "cfa": info[3],
        "power_up": power_sequences[0],
        "power_down": power_sequences[1],
        "modes": modes, "init": init,
        "stream_on": seq[0], "stream_off": seq[1],
        "group_on": seq[2], "group_off": seq[3],
    }


def write_reg_list(lines, name, regs):
    lines.append(f"static const struct raphael_sensor_reg {name}[] = {{")
    for addr, value, width, delay in regs:
        lines.append(f"\t{{ CCI_REG{width*8}(0x{addr:04x}), 0x{value:0{width*2}x}, {delay} }},")
    lines.append("};")
    lines.append("")


def main():
    global ROOT, DEST

    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--vendor-dir", required=True, type=Path,
                        help="directory containing the seven pinned CamX BIN files")
    parser.add_argument("--output", type=Path,
                        default=Path(__file__).resolve().parent.parent /
                        "drivers/media/i2c/raphael-sensor-tables.c",
                        help="generated C output (defaults to the kernel source tree)")
    args = parser.parse_args()
    ROOT = args.vendor_dir
    DEST = args.output
    data = {name: extract_sensor(tag) for name, tag in SOURCES.items()}
    for name, alternate in ALTERNATES.items():
        other = extract_sensor(alternate)
        first = data[name]
        for key in ("i2c_8bit", "chip_id_register", "chip_id",
                    "frame_length_register", "exposure_register",
                    "analogue_gain_register", "max_analog_gain",
                    "vertical_offset", "cfa", "power_up", "power_down",
                    "modes", "init", "stream_on", "stream_off",
                    "group_on", "group_off"):
            assert first[key] == other[key], (name, key)
    imx = data["imx586"]
    ov = extract_sensor("raphael_luxvisions_ov8856")
    assert imx["modes"][0]["regs"][:3] == (
        (0x0112, 0x0a, 1, 0), (0x0113, 0x0a, 1, 0),
        (0x0114, 0x03, 1, 0))
    assert [(a, v) for a, v, _, _ in ov["modes"][0]["regs"]
            if a in (0x3808, 0x3809, 0x380a, 0x380b)] == [
                (0x3808, 0x0c), (0x3809, 0xc0),
                (0x380a, 0x09), (0x380b, 0x90)]
    for item in data.values():
        assert item["stream_off"][-1][1] == 0

    # CamX power tuples are (delay after action in ms, configType, value).
    # Type values: MCLK=0, VANA=1, VDIG=2, VIO=3, CUSTOM1=6, RESET=8.
    main_up = ((5, 1, 1), (0, 6, 1), (0, 2, 1), (0, 3, 1),
               (0, 0, 19200000), (10, 8, 1), (2, 8, 0))
    common_up = ((5, 1, 1), (0, 3, 1), (0, 2, 1),
                 (0, 0, 19200000), (10, 8, 1), (2, 8, 0))
    main_down = ((2, 0, 0), (0, 3, 0), (0, 2, 0),
                 (0, 6, 0), (0, 1, 0))
    common_down = ((1, 0, 0), (1, 2, 0), (0, 3, 0), (0, 1, 0))
    expected_sensor_properties = {
        "imx586": (3, 64.0, 48),
        "ov8856": (0, 15.5, 8),
        "s5k3l6": (1, 16.0, 20),
        "s5k3t2": (1, 16.0, 20),
    }
    for name, item in data.items():
        assert item["power_up"] == (main_up if name == "imx586"
                                    else common_up)
        prefix = main_down if name == "imx586" else common_down
        assert item["power_down"][:len(prefix)] == prefix
        assert (item["cfa"], item["max_analog_gain"],
                item["vertical_offset"]) == expected_sensor_properties[name]

    # IMX586 mode 2 claims 4000x2250 at 30 fps, while its output clock
    # can carry only 174 MPix/s. Keep the table in the source BIN and
    # exclude the contradictory mode from the V4L2 mode list.
    bad_mode = data["imx586"]["modes"][2]
    assert bad_mode["width"] == 4000 and bad_mode["height"] == 2250
    assert bad_mode["output_pixel_clock"] * 100 < (bad_mode["width"] *
           bad_mode["height"] * bad_mode["fps_x100"])
    data["imx586"]["modes"] = [mode for mode in data["imx586"]["modes"]
                                if mode["source_mode"] != 2]
    for item in data.values():
        for mode in item["modes"]:
            assert mode["output_pixel_clock"] * 100 >= (mode["width"] *
                   mode["height"] * mode["fps_x100"])

    lines = [
        "// SPDX-License-Identifier: GPL-2.0-only",
        "/*",
        " * Raphael sensor register settings reconstructed from public CamX module",
        " * data. The source binaries are not loaded by this kernel driver.",
        " * Source repository: Evolution-X-Devices/vendor_xiaomi_raphael (bka).",
        " */",
        '#include <linux/kernel.h>',
        '#include <linux/media-bus-format.h>',
        '#include "raphael-sensor.h"',
        "",
    ]
    cfa_codes = {
        0: "MEDIA_BUS_FMT_SBGGR10_1X10",
        1: "MEDIA_BUS_FMT_SGRBG10_1X10",
        2: "MEDIA_BUS_FMT_SGBRG10_1X10",
        3: "MEDIA_BUS_FMT_SRGGB10_1X10",
    }
    # Minimum exposure and gain register codes come from the same-chip
    # Linux drivers: this tree's ov8856.c, the public S5K3L6/S5K3T2
    # linux-media RFCs, and the Sony IMX586 gain equation. CamX exposes
    # maxAnalogGain as a real gain ratio, not a register code. The variant
    # limits below remain conservative until they can be tested on Raphael.
    definitions = {
        "imx586": (6, 0, 0, True, 2, 0),
        "ov8856": (6, 128, 128, False, 3, 4),
        "s5k3l6": (2, 0x20, 0x20, False, 2, 0),
        "s5k3t2": (8, 0x20, 0x20, False, 2, 0),
    }
    for name, item in data.items():
        (exp_min, gain_min, gain_def,
         custom1, exposure_bytes, exposure_shift) = definitions[name]
        code = cfa_codes[item["cfa"]]
        exp_margin = item["vertical_offset"]
        gain_ratio = item["max_analog_gain"]
        if name == "imx586":
            # Sony IMX586: ratio = 1024 / (1024 - code).
            gain_max = 1024 - round(1024 / gain_ratio)
        elif name == "ov8856":
            # OV8856's 0x3508 code uses 0x80 for unity gain.
            gain_max = round(gain_ratio * 128)
        else:
            # Both Samsung parts use 0x20 for unity analogue gain.
            gain_max = round(gain_ratio * 32)
        assert gain_min <= gain_def <= gain_max <= 0xffff
        lines += [
            f"/* {item['path'].name}",
            f" * SHA256 {item['sha256']}",
            " */",
        ]
        for suffix in ("init", "stream_on", "stream_off", "group_on", "group_off"):
            write_reg_list(lines, f"{name}_{suffix}_regs", item[suffix])
        if name == "imx586":
            lines += ["/* BIN mode 2 is omitted: CSI rate < claimed RAW10 active rate. */", ""]
        for mode in item["modes"]:
            write_reg_list(lines, f"{name}_mode{mode['source_mode']}_regs", mode["regs"])
        frequencies = list(dict.fromkeys(m["link_freq"] for m in item["modes"]))
        lines.append(f"static const s64 {name}_link_frequencies[] = {{")
        for freq in frequencies:
            lines.append(f"\t{freq},")
        lines += ["};", "", f"static const struct raphael_sensor_mode {name}_modes[] = {{"]
        for mode in item["modes"]:
            lines += [
                "\t{",
                f"\t\t.width = {mode['width']},",
                f"\t\t.height = {mode['height']},",
                f"\t\t.hts = {mode['hts']},",
                f"\t\t.vts = {mode['vts']},",
                f"\t\t.fps_x100 = {mode['fps_x100']},",
                f"\t\t.pixel_rate = {mode['pixel_rate']},",
                f"\t\t.link_freq = {mode['link_freq']},",
                f"\t\t.link_freq_index = {frequencies.index(mode['link_freq'])},",
                f"\t\t.settings = {{ {name}_mode{mode['source_mode']}_regs, ARRAY_SIZE({name}_mode{mode['source_mode']}_regs) }},",
                "\t},",
            ]
        lines += ["};", "", f"const struct raphael_sensor_variant raphael_{name} = {{"]
        lines += [
            f'\t.name = "{name}",',
            f"\t.chip_id_register = 0x{item['chip_id_register']:04x},",
            f"\t.chip_id = 0x{item['chip_id']:04x},",
            f"\t.frame_length_register = 0x{item['frame_length_register']:04x},",
            f"\t.vts_max = 0x{'7fff' if name == 'ov8856' else 'ffff'},",
            f"\t.exposure_register = 0x{item['exposure_register']:04x},",
            f"\t.exposure_data_bytes = {exposure_bytes},",
            f"\t.exposure_shift = {exposure_shift},",
            f"\t.analogue_gain_register = 0x{item['analogue_gain_register']:04x},",
            f"\t.gain_min = {gain_min},",
            f"\t.gain_max = {gain_max},",
            f"\t.gain_default = {gain_def},",
            f"\t.exposure_min = {exp_min},",
            f"\t.exposure_margin = {exp_margin},",
            f"\t.custom1_supply = {'true' if custom1 else 'false'},",
            f"\t.mbus_code = {code},",
            f"\t.link_frequencies = {name}_link_frequencies,",
            f"\t.num_link_frequencies = ARRAY_SIZE({name}_link_frequencies),",
            f"\t.modes = {name}_modes,",
            f"\t.num_modes = ARRAY_SIZE({name}_modes),",
        ]
        for member, suffix in (("init", "init"), ("stream_on", "stream_on"),
                               ("stream_off", "stream_off"),
                               ("group_hold_on", "group_on"),
                               ("group_hold_off", "group_off")):
            lines.append(f"\t.{member} = {{ {name}_{suffix}_regs, ARRAY_SIZE({name}_{suffix}_regs) }},")
        lines += ["};", ""]
    DEST.write_text("\n".join(lines))
    print(f"wrote {DEST}: {len(lines)} lines, {sum(len(x['modes']) for x in data.values())} modes")
    for name, item in data.items():
        print(name, "I2C 7bit", hex(item["i2c_8bit"] >> 1), "ID", hex(item["chip_id"]))


if __name__ == "__main__":
    main()
