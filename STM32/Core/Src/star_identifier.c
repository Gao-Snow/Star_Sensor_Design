/**
@file star_identifier.c
@brief ��ͼʶ��ģ��ʵ�֣�������ƥ�䣩
���������ͼƬ��ʶ��ͼƬ���ǵ����Ҫ�߼��ǣ�
1.�����������ȷ���ǵ����� 
2.Ȼ�����ѡ�������ǵ����������
3.��Ϊ�������漰�߳����ڽǽǶȣ��ر����ڽǽǶȣ�����۲�ǶȺ�Զ�������仯��������Ϊʶ�������λ�õ�����
4.��ÿ�ֿ��ܵ���������϶���¼���ڽ�����ֵ��64λ�������������� tri_keys[]�� tri_indices[] 
5.��ÿ���۲������Σ��� TriangleFeature���ݿ��в�����ͬ key�����ҵ�����Ϊ�����������Ǳ���ĳ�������Ρ��������ơ�
Ϊ�������۲��Ƿֱ�ͶһƱ������Ӧ���Ǳ� ID
6.ÿ�Ź۲���ά��һ����ѡ ID �б�ѡȡ��Ʊ �� VOTE_THRESHOLD�����Ʊ ID�����ƥ����

 */
 
 
/*
����˵����
1.�Ǳ�Star Catalog�����ṹ�����ݱ�ÿ����¼����һ�ź����ڿռ��еķ�������ȣ������� 
	id���ǵ�Ψһ��ʶ������ Hipparcos ID��
	vec[3]����������������ϵ�еĵ�λ�����������ο�ϵ��
	magnitude�����ǵȣ����ȣ���ѡ�� 
	
2.��������Ǳ�������п��ܵ������Σ��������������Ҳ������·�����С��������		
	��������ǵȣ�ֻ��¼���ǣ���ǰ 200 �ţ���
	�������Ǿֻࣺ���ǽǾ���ĳ����Χ�ڵ������Σ�����̫���̫�ۣ�
	�������� + ѹ���洢�����������ݿ���Դ洢�� Flash �У��������
	���� + ��ϣ���� 64 λ�����渡��Ƚϣ���ʡ�ռ�Ͳ���ʱ�� 
	
3.ͶƱ����ϸ���̣� 
	��ÿ�Ź۲��ǣ������ i�ţ���ÿ�������ҵ�һ��ƥ��������Σ���Ϊ�����Ź۲��Ǹ���ͶһƱ����Ӧ�������Ǳ��ǡ�
	���ԣ�
	�۲��� 0 �ĺ�ѡ�б�[ (id=123�������ǣ�, votes=2), (id=456������ģ�, votes=1) ]
	�۲��� 1 �ĺ�ѡ�б�[ (id=123, votes=3), (id=789, votes=1) ]
	�۲��� 2 �ĺ�ѡ�б�[ (id=123, votes=2), (id=999, votes=1) ]
	ѡȡ��Ʊ�������Ϊ����ƥ��
	��ÿ�Ź۲��ǣ������ĺ�ѡ�б����� Ʊ����ߵ� ID
	����Ʊ�� �� VOTE_THRESHOLD��Ĭ�� 2��������ܸ� ID ��Ϊƥ����
	������Ϊ 0��δƥ�䣩
4.ͶƱƽƱ���
	�������������������߹۲��� A �� B ��ͼ���кܽ��������Ǹ��ݸ��Ե�������ƥ�䣬���Ʊ�������Ǳ��е������ǣ���ƽƱ
	���� A �� B ��ƥ�䵽������
	��Ĭ�������Ƕ��������ǣ�����ִ�к���������ֻҪƥ����Ҫ��ͨ��������̬������֤
*/

#include "star_identifier.h"
#include <stdlib.h>
#include <string.h>
#include <math.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846f
#endif

/* �ڲ����� */
#define MAX_OBS_STARS 50          ///< ���۲��������� pipeline ����һ�£�
#define MAX_TRIANGLES_PER_OBS 1000 ///< ���۲�������������������ƣ�
#define VOTE_THRESHOLD 3           ///< ͶƱ��ֵ�����ٻ�ü�Ʊ��

/* �Ƕ��������ȣ�ÿ���Ƕȳ��� 1000 ��ȡ������ϳ� 64 λ�� */
#define ANGLE_QUANT_FACTOR 1000.0f

/**
 * @brief ʶ����ʵ���ṹ������ʵ�֣�
 */
struct StarIdentifier {
    const NavStarEntry* catalog;
    int catalog_size;
    const TriangleFeature* db;
    int db_size;
};

/* ========== �ڲ��������� ========== */

/**
 * @brief ����������λ����֮��ļнǣ����ȣ�
 */
static float angle_between(const float a[3], const float b[3]) {
    float dot = a[0]*b[0] + a[1]*b[1] + a[2]*b[2];//������õ�cos�� 
    if (dot > 1.0f) dot = 1.0f;
    if (dot < -1.0f) dot = -1.0f;
/*������������ֹ�������� acosf������
�����ϣ�dot �� [-1, 1]�����ڸ��㾫����dot���ܱ�� 1.0000002�� -1.0000001
���ʱreturn acosf(x)�� x > 1�� x < -1ʱ������������ δ������Ϊ��NaN��
*/ 
    return acosf(dot);//����dot����������arccos��������Ƕ� 
}

/**
 * @brief �������������������������Ƕ�������
 */
static uint64_t make_feature_key(float a1, float a2, float a3) {
    /*
    �����ε�����ֵ���������ʽ�洢�����������ڽǵ�˳�������ͬ����ͬһ�������ο�����������������
	Ϊ�˷�ֹ˳�����⵼���ظ����㣬����һ��Ψһ�ġ��ɱȽϵ������������ڿ��ٲ���������
	����ʹ�ýǶ���������������������һ�������Σ������ڽǵ����ǰ�����С�����˳�򱻴洢
	*/
    float angles[3] = {a1, a2, a3};
    if (angles[0] > angles[1]) { float t = angles[0]; angles[0] = angles[1]; angles[1] = t; }
    if (angles[1] > angles[2]) { float t = angles[1]; angles[1] = angles[2]; angles[2] = t; }
    if (angles[0] > angles[1]) { float t = angles[0]; angles[0] = angles[1]; angles[1] = t; }

    // �������Ѹ���Ƕ�ת�����������ڴ洢�ͱȽϡ�
	/*
	�������һ�����Ƶ�
	�ڱ�����ͷ�����ǹ涨angle_quant_factor����ϵ��Ϊ1000
	angles[0]���Ի���Ϊ��λ���ڽǣ����� 1000 �󣬰ѻ���ת�ɺ����ȣ�milliradian, mrad����������1 ������ �� 0.0573��
	�������ڽǷ�Χ��0<��<��(��0��?180)���û��ȱ�ʾ��0<��<3.14159265?rad
	�������ֵ��Χ
	q=�ȡ�1000
	��Сֵ��0
	���ֵ��3.14159��1000=3141.59=3141
	���ԣ�ÿ������ֵ q�ķ�Χ�� 0 ~ 3141���� 3142 �ֿ���ֵ
	����ʹ��21λ�����ƣ�����ʾ��ͬ���ܣ�2^21=2097152ԶԶ����3000��
	��һ��������Ҫ����Щ��ֵͬװ��64λ���У���ȷ��һ�ַ�ʽʹ��64λװ������ 21λֵ��
	��������q0,q1,q2�ֱ������������
	q0������ 42 λ �� ռ�� bit 42�C62���� 21 λ��
	q1������ 21 λ �� ռ�� bit 21�C41���� 21 λ��
	q2������λ �� ռ�� bit 0�C20���� 21 λ��
	Ȼ��ʹ��return ((uint64_t)q0 << 42) | ((uint64_t)q1 << 21) | q2;����ȷ��λ����λ�þ���ȷ
	*/
    uint32_t q0 = (uint32_t)(angles[0] * ANGLE_QUANT_FACTOR);
    uint32_t q1 = (uint32_t)(angles[1] * ANGLE_QUANT_FACTOR);
    uint32_t q2 = (uint32_t)(angles[2] * ANGLE_QUANT_FACTOR);

    // ��ϳ� 64 λ����ÿ���� 21 λ���� 63 λ��
    return ((uint64_t)q0 << 42) | ((uint64_t)q1 << 21) | q2;
}

/**
 * @brief �����ݿ��в���ƥ�������������ɨ�裬С�����ݿ����ã�
 * 
 * ע�⣺�����ݿ�ܴ�Ӧʹ�ù�ϣ������Ϊ�˼򻯣��������Բ��ҡ�
 * Ƕ��ʽ���������������ݿ�ͨ�����󣨼�ǧ����������ɨ��ɽ��ܡ�
 */
 
 
static int find_matches_in_db(const TriangleFeature* db, int db_size,
                              uint64_t key, uint32_t out_ids[][3], int max_out) {
    int count = 0;

    // 提取观测三角形的三个角度（21位数据）
    int32_t obs_q0 = (key >> 42) & 0x1FFFFF;
    int32_t obs_q1 = (key >> 21) & 0x1FFFFF;
    int32_t obs_q2 = key & 0x1FFFFF;

    // 允许的误差范围：2个单位，相当于 0.002 弧度 (约 0.11 度)
    // 根据实际情况，如果依然匹配不上，可以把 tolerance 放大到 3 或 4
    int32_t tolerance = 2;

    for (int i = 0; i < db_size && count < max_out; ++i) {
        uint64_t db_key = db[i].feature_key;

        // 提取数据库中的三个角度
        int32_t db_q0 = (db_key >> 42) & 0x1FFFFF;
        int32_t db_q1 = (db_key >> 21) & 0x1FFFFF;
        int32_t db_q2 = db_key & 0x1FFFFF;

        // 检查三个角度是否都在容差范围内
        if (abs(obs_q0 - db_q0) <= tolerance &&
            abs(obs_q1 - db_q1) <= tolerance &&
            abs(obs_q2 - db_q2) <= tolerance) {

            out_ids[count][0] = db[i].star_ids[0];
            out_ids[count][1] = db[i].star_ids[1];
            out_ids[count][2] = db[i].star_ids[2];
            count++;
        }
    }
    return count;
}
/*
���ܴ��ڶ��ƥ�䣺
1.��ͬ�������ǿ��ܹ������Ƶ������Σ�������ϡ���ǳ���
2.�������²�ͬ��ʵ������ӳ�䵽ͬһ�� key
���Լ�ʹ�����ͬҲ���أ�����ͶƱ���ƴ��� 
*/


/* ========== ���� API ========== */

StarIdentifier* star_identifier_create(
    const NavStarEntry* catalog,
    int catalog_size,
    const TriangleFeature* db,
    int db_size)
{
    // 分配一个静态的实例，避免内存分配
    static StarIdentifier dummy_inst;
    dummy_inst.catalog = catalog;  // 这里可以传入实际指针，但不会使用
    dummy_inst.catalog_size = catalog_size;
    dummy_inst.db = db;
    dummy_inst.db_size = db_size;
    return &dummy_inst;
}

void star_identifier_destroy(StarIdentifier* inst) {
    free(inst);
}

IdentifierStatus star_identifier_match(
    const StarIdentifier* inst,
    const float obs_vectors[][3],
    int num_obs,
    uint32_t matches_out[],
    int* match_count_out)
{
    if (!inst || !obs_vectors || !matches_out || !match_count_out) {
        return IDENTIFIER_ERROR_INVALID_PARAM;
    }
    if (num_obs < 3 || num_obs > MAX_OBS_STARS) {
        return IDENTIFIER_ERROR_NOT_ENOUGH_STARS;
    }

    *match_count_out = 0;

    /* 1. �������й۲������� */

    /* 1. 提取所有观测三角形 */
        int tri_count = 0;
        static uint64_t tri_keys[MAX_TRIANGLES_PER_OBS];
        static int tri_indices[MAX_TRIANGLES_PER_OBS][3];   // 记录三角形对应的观测星索引

        // 必须有 i, j, k 三层嵌套循环！
        for (int i = 0; i < num_obs - 2; ++i) {
            for (int j = i + 1; j < num_obs - 1; ++j) {
                for (int k = j + 1; k < num_obs; ++k) {

                    // 达到上限，干净利落地跳出三重循环
                    if (tri_count >= MAX_TRIANGLES_PER_OBS) goto TRI_EXTRACT_DONE;

                    // 提取三角形的三条边（角距离弧度）
                    float a = angle_between(obs_vectors[i], obs_vectors[j]);
                    float b = angle_between(obs_vectors[i], obs_vectors[k]);
                    float c = angle_between(obs_vectors[j], obs_vectors[k]);

                    // 抛弃球面内角，直接把三条边长打包成 Key！极为稳定！
                    uint64_t key = make_feature_key(a, b, c);

                    tri_keys[tri_count] = key;
                    tri_indices[tri_count][0] = i;
                    tri_indices[tri_count][1] = j;
                    tri_indices[tri_count][2] = k;
                    tri_count++;
                }
            }
        }
        TRI_EXTRACT_DONE: // 跳出标签

        if (tri_count == 0) {
            return IDENTIFIER_ERROR_NO_MATCH;
        }

    /* 2. ͶƱ���󣨹۲������� -> �Ǳ�ID ��ͶƱ������ 
    �۲������� <= MAX_OBS_STARS���Ǳ�ID���ܺܶ࣬��ƥ��ĺ�ѡ���ޡ�
    ����ʹ�ö�̬ӳ�䣺��ÿ���۲��ǣ���¼һ����ѡID����Ʊ����
    �����������ʹ�ù̶���С�������¼ÿ���۲��Ƕ�Ӧ�ĺ�ѡ�����Ǳ�ID���ܴܺ�
    ʵ�õķ�����ʹ�ù�ϣ����Ϊ�˼򻯣����ǲ�����ʱ����+���Բ��Һ�ѡ��
    ����ʵ��һ�ּ򻯣���ÿ���۲������Σ�����ҵ�ƥ�䣬��Ϊ�����Ƕ�Ӧ���Ǳ�IDͶƱ��
    ����ÿ���۲��ǣ�ѡȡ��Ʊ��ߵ�ID��*/

    // �����ѡ�ṹ
    typedef struct {
        uint32_t id;
        int votes;
    } Candidate;

    // Ϊÿ���۲���ά��һ����ѡ�б�������10����ѡ��
    #define MAX_CANDIDATES 10
    static Candidate candidates[MAX_OBS_STARS][MAX_CANDIDATES];
    static int cand_count[MAX_OBS_STARS];

    // ��ʼ����ѡ�б�Ϊ��
    for (int i = 0; i < num_obs; ++i) {
        cand_count[i] = 0;
    }

    // �������й۲������Σ�����ƥ��
    for (int t = 0; t < tri_count; ++t) {
        // �����ݿ��в���
        static uint32_t matches[100][3];  // �������100��ƥ��
        int num_matches = find_matches_in_db(inst->db, inst->db_size,
                                             tri_keys[t], matches, 100);

        for (int m = 0; m < num_matches; ++m) {
            // �������ƥ����Ǳ������� (ids[0], ids[1], ids[2])
            // ����۲������ε�˳�����Ǳ�������˳���Ӧ����Ҫ�����������⣩
            // ʵ���У���������ʹ�õ��������ĽǶȣ�˳���Ѿ��̶���
            // ��˿���ֱ�Ӷ�Ӧ���۲��� i ��Ӧ�Ǳ�ID matches[m][0]���ȵȡ�
            // ����Ҫע�⣬����������󣬱ߵĶ�Ӧ��ϵ��һ�����֣���ͨ����������ܵĶ�Ӧ��ֱ�Ӷ�Ӧ��
            // ����򻯴���ֱ�Ӱ�˳���Ӧ��
            for (int k = 0; k < 3; ++k) {
                int obs_idx = tri_indices[t][k];
                uint32_t cat_id = matches[m][k];

                // �ں�ѡ�б��в����Ƿ��Ѵ��ڸ� cat_id
                int found = -1;
                for (int c = 0; c < cand_count[obs_idx]; ++c) {
                    if (candidates[obs_idx][c].id == cat_id) {
                        found = c;
                        break;
                    }
                }
                if (found >= 0) {
                    candidates[obs_idx][found].votes++;
                } else {
                    // ����º�ѡ
                    if (cand_count[obs_idx] < MAX_CANDIDATES) {
                        candidates[obs_idx][cand_count[obs_idx]].id = cat_id;
                        candidates[obs_idx][cand_count[obs_idx]].votes = 1;
                        cand_count[obs_idx]++;
                    }
                }
            }
        }
    }

    /* 3. ѡ�����ƥ�� */
    int matched = 0;
    for (int i = 0; i < num_obs; ++i) {
        if (cand_count[i] == 0) {
            matches_out[i] = 0xFFFFFFFF; // 0 ��ʾδƥ��
            continue;
        }
        // ��Ʊ����ߵĺ�ѡ
        int best_idx = 0;
        int best_votes = candidates[i][0].votes;
        for (int c = 1; c < cand_count[i]; ++c) {
            if (candidates[i][c].votes > best_votes) {
                best_votes = candidates[i][c].votes;
                best_idx = c;
            }
        }
        if (best_votes >= VOTE_THRESHOLD) {
            matches_out[i] = candidates[i][best_idx].id;
            matched++;
        } else {
            matches_out[i] = 0xFFFFFFFF;
        }
    }

    *match_count_out = matched;
    return (matched >= 3) ? IDENTIFIER_SUCCESS : IDENTIFIER_ERROR_NO_MATCH;
}
