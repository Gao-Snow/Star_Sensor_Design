import os
import math
import csv

# ================= 配置路径 =================
RAW_DATA_DIR = r"D:\1StarTracker\TychoRowData"
# 【修改】：全部输出到当前 Python 目录，不再直接改动单片机源码
OUT_DIR = r"D:\1StarTracker\CatalogPython"

# 亮度过滤阈值
MAG_THRESHOLD = 3.5
# 视场角限制：35度对角线对应约 25度最大边长 (0.436弧度)
MAX_EDGE_RAD = 25.0 * math.pi / 180.0


def parse_tycho2_raw():
    stars = []
    print("开始解析 Tycho-2 原始数据...")
    for i in range(20):
        filename = f"tyc2_{i:02d}.dat"
        filepath = os.path.join(RAW_DATA_DIR, f"tyc2.dat.{i:02d}", filename)

        if not os.path.exists(filepath):
            continue

        with open(filepath, 'r', encoding='ascii') as f:
            for line in f:
                parts = line.split('|')
                if len(parts) < 20: continue

                tyc_ids = parts[0].split()
                if len(tyc_ids) != 3: continue

                # 【极其关键的修复】：VTmag 是第 19 列（索引从0开始）
                vt_str = parts[19].strip()
                ra_str = parts[2].strip()
                dec_str = parts[3].strip()

                if not vt_str or not ra_str or not dec_str: continue

                vt = float(vt_str)
                if vt <= MAG_THRESHOLD:
                    ra = float(ra_str)
                    dec = float(dec_str)

                    ra_rad = math.radians(ra)
                    dec_rad = math.radians(dec)
                    x = math.cos(dec_rad) * math.cos(ra_rad)
                    y = math.cos(dec_rad) * math.sin(ra_rad)
                    z = math.sin(dec_rad)

                    stars.append({
                        'tyc1': int(tyc_ids[0]),
                        'tyc2': int(tyc_ids[1]),
                        'tyc3': int(tyc_ids[2]),
                        'ra': ra, 'dec': dec, 'vt': vt,
                        'x': x, 'y': y, 'z': z
                    })

    stars.sort(key=lambda s: s['vt'])
    print(f"解析完成！共提取出 {len(stars)} 颗亮于 {MAG_THRESHOLD} 等的恒星。")
    return stars


def write_csv(stars):
    csv_path = os.path.join(OUT_DIR, "tycho2_bright_stars.csv")
    with open(csv_path, 'w', newline='', encoding='utf-8') as f:
        writer = csv.writer(f)
        writer.writerow(['TYC1', 'TYC2', 'TYC3', 'RA_deg', 'Dec_deg', 'VTmag'])
        for s in stars:
            writer.writerow([s['tyc1'], s['tyc2'], s['tyc3'], s['ra'], s['dec'], s['vt']])
    print(f"已生成 CSV 星表: {csv_path}")


def build_c_database(stars):
    nav_entries = []
    for s in stars:
        tyc1_2 = (s['tyc1'] << 20) | s['tyc2']
        ra_mdeg = int(s['ra'] * 1000)
        dec_mdeg = int(s['dec'] * 1000)
        vt_mmag = int(s['vt'] * 100)
        x_q15 = int(round(s['x'] * 32768.0))
        y_q15 = int(round(s['y'] * 32768.0))
        z_q15 = int(round(s['z'] * 32768.0))

        nav_entries.append(
            f"    {{ {tyc1_2}ULL, {s['tyc3']}, {ra_mdeg}, {dec_mdeg}, {vt_mmag}, {x_q15}, {y_q15}, {z_q15} }}"
        )

    triangles = []
    for i in range(len(stars)):
        for j in range(i + 1, len(stars)):
            for k in range(j + 1, len(stars)):
                v1 = (stars[i]['x'], stars[i]['y'], stars[i]['z'])
                v2 = (stars[j]['x'], stars[j]['y'], stars[j]['z'])
                v3 = (stars[k]['x'], stars[k]['y'], stars[k]['z'])

                dot_a = max(-1.0, min(1.0, v1[0] * v2[0] + v1[1] * v2[1] + v1[2] * v2[2]))
                dot_b = max(-1.0, min(1.0, v1[0] * v3[0] + v1[1] * v3[1] + v1[2] * v3[2]))
                dot_c = max(-1.0, min(1.0, v2[0] * v3[0] + v2[1] * v3[1] + v2[2] * v3[2]))

                a, b, c = math.acos(dot_a), math.acos(dot_b), math.acos(dot_c)

                if max(a, b, c) <= MAX_EDGE_RAD:
                    edges = sorted([a, b, c])
                    q0 = int(edges[0] * 1000.0)
                    q1 = int(edges[1] * 1000.0)
                    q2 = int(edges[2] * 1000.0)

                    key = (q0 << 42) | (q1 << 21) | q2
                    triangles.append((key, i, j, k))

    triangles.sort(key=lambda t: t[0])

    cat_c_path = os.path.join(OUT_DIR, "star_catalog_nav.c")
    tri_c_path = os.path.join(OUT_DIR, "triangle_db.c")
    inc_h_path = os.path.join(OUT_DIR, "star_catalog_nav.h")

    with open(inc_h_path, 'w', encoding='utf-8') as f:
        f.write("#ifndef STAR_CATALOG_NAV_H\n#define STAR_CATALOG_NAV_H\n\n#include <stdint.h>\n\n")
        f.write(
            "typedef enum { CATALOG_SUCCESS = 0, CATALOG_ERROR_NOT_FOUND = -1, CATALOG_ERROR_INVALID_ID = -2 } CatalogStatus;\n\n")
        f.write("CatalogStatus catalog_get_vector_by_index(uint16_t index, float vec_out[3]);\n\n")
        f.write(f"#define NAV_STAR_COUNT {len(stars)}\n")
        f.write(f"#define TRIANGLE_COUNT {len(triangles)}\n\n")
        f.write(
            "typedef struct {\n    uint64_t tyc1_2;\n    uint8_t  tyc3;\n    int32_t  ra_mdeg;\n    int32_t  dec_mdeg;\n    uint16_t vt_mmag;\n    int16_t  x, y, z;\n} NavStarEntry;\n\n")
        f.write("typedef struct {\n    uint64_t feature_key;\n    uint16_t star_ids[3];\n} TriangleFeature;\n\n")
        f.write("extern const NavStarEntry g_nav_star_catalog[NAV_STAR_COUNT];\n")
        f.write("extern const TriangleFeature g_triangle_db[TRIANGLE_COUNT];\n\n#endif\n")

    with open(cat_c_path, 'w', encoding='utf-8') as f:
        f.write('#include "star_catalog_nav.h"\n#include <math.h>\n\n')
        f.write(f"const NavStarEntry g_nav_star_catalog[{len(stars)}] = {{\n")
        f.write(",\n".join(nav_entries))
        f.write("\n};\n\n")
        f.write("""CatalogStatus catalog_get_vector_by_index(uint16_t index, float vec_out[3]) {
    if (index >= NAV_STAR_COUNT) return CATALOG_ERROR_INVALID_ID;
    const NavStarEntry* star = &g_nav_star_catalog[index];
    const float scale = 1.0f / 32768.0f;
    vec_out[0] = star->x * scale;
    vec_out[1] = star->y * scale;
    vec_out[2] = star->z * scale;
    float norm = sqrtf(vec_out[0]*vec_out[0] + vec_out[1]*vec_out[1] + vec_out[2]*vec_out[2]);
    if (norm > 1e-6f) { vec_out[0] /= norm; vec_out[1] /= norm; vec_out[2] /= norm; }
    return CATALOG_SUCCESS;
}\n""")

    with open(tri_c_path, 'w', encoding='utf-8') as f:
        f.write('#include "star_catalog_nav.h"\n\n')
        f.write(f"const TriangleFeature g_triangle_db[{len(triangles)}] = {{\n")
        for t in triangles:
            f.write(f"    {{ {t[0]}ULL, {{ {t[1]}, {t[2]}, {t[3]} }} }},\n")
        f.write("};\n")

    print(f"✅ 生成成功！请前往 {OUT_DIR} 手动复制粘贴到 STM32 工程中。")
    print(f"👉 星星数量: {len(stars)} 颗 | 三角形数量: {len(triangles)} 个")


if __name__ == "__main__":
    stars = parse_tycho2_raw()
    if stars:
        write_csv(stars)
        build_c_database(stars)