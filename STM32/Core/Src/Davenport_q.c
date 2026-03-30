/**
 * @file davenport.c
 * @brief ��̬����ģ��ʵ�֣�Davenport q ������
 */

#include "Davenport_q.h"
#include <math.h>
#include <string.h>
//�����Ҫ��

/* ========================= �ڲ����� ========================= */
#define MAT4_SIZE 16   // 4x4 ����Ԫ�ظ���
#define SINGULARITY_THRESHOLD 0.999999f  
/* ŷ������������ֵ
ԭ���ǣ�ʹ��ŷ���Ǳ�ʾ��̬ʱ���ܻ����һ�Զ������
�ҵ�����������ģ�ŷ����˵���ǣ����ǰ���Լ��˳��ֱ����������в�ͬ�Ƕȵ���ת����������ǰ��̬����ڲο�ϵ�ı仯 
����������ת˳���йأ���ʹ�ǶȲ��䣬������ת˳��Ҳ��ʹ����̬��һ��
��һ���棬������ת���ǹ̶��ڱ����������ϵģ����Ե�һ������ת��ʱ������������Ҳ����ת����������᲻�ܴ���ǰ�������ת 
������תx�ᣬ����תy��90��ʹ��z����x���غϣ��ͻᵼ�¶�ʧһ�����ɶȣ���ʱ��ͬ�� yaw �� roll ��Ͽ��ܲ�����ͬ����̬�������̬������Ψһ 
��˱���Ʋ���4Ԫ����������̬�������Ͳ��������������⡣
����Ҫ���ǵ��� ����������ܻᴫ������վ������ʱŷ�������Ϊֱ�� 
��ô���м��ת�������У�������Ҫ�����ĳЩ�жϣ��������Ƿ�ӽ�������̬������ʱ�Ϳ�������һ����ֵ
������Ԫ��תŷ����ʱ��pitch �ǽӽ� ��90�� ʱ������ roll �� yaw �������ֵ���ȶ������Խӽ���������� arctan2 �Ĳ������ˣ���
Ϊ����ǰ������ֽӽ��������������Լ����Ԫ������ת������ĳ�����Ƿ񳬹���ֵ��
�����ֵ > 0.999999 ʱ������Ϊ���ǳ��ӽ�����������

NASA�ֲ����ᵽ��һ��ʹ��DAVENPORT�㷨����4Ԫ����ʾ��̬�ķ��������о��д��� 
if (��� > SINGULARITY_THRESHOLD) {
    // �ӽ����������������֧����
} else {
    // ��������
}
 
*/

/* ========================= �ڲ����ߺ��� ========================= */

/** ������� */
static float vec_dot(const DavenportVector3* a, const DavenportVector3* b) {
    return a->x * b->x + a->y * b->y + a->z * b->z;
}//����ʹ��vec_dot�������е�����㣬��ͷstatic��ʾ�������ֻ�����ڲ����ã�pipeline�޷����ʣ������� 

/** ������� */
static DavenportVector3 vec_cross(const DavenportVector3* a, const DavenportVector3* b) {
    DavenportVector3 v;
    v.x = a->y * b->z - a->z * b->y;
    v.y = a->z * b->x - a->x * b->z;
    v.z = a->x * b->y - a->y * b->x;
    return v;
}
//�����ǲ����ʹ�ýṹ��a��b�����Ƿֱ�洢�Ÿ��Ե�xyz���꣬���ò���ľ�������Ϳ��Եõ������û������ 

/** 3x3 ����ļ� */
static float mat_trace(const float B[3][3]) {
    return B[0][0] + B[1][1] + B[2][2];
}
//���Ǽ������ļ��������Խ�Ԫ���ۼӣ�û������ 

/**
 * @brief ������̬������� B = sum_i w_i * (b_i * r_i^T)
�������ڸ��ݹ۲�����b_i���ο�����r_i��Ȩ�أ�����B���� 
 */
 
static void compute_B_matrix(float B[3][3],
                             const DavenportVector3 body[],
                             const DavenportVector3 ref[],
                             const float weights[],
//������ǰ���constant��ȷ����ǰ����ֻ���������ߴ�������������ں����ڲ����ᱻ�޸�
                             int n) {
    // ��B[][]������г�ʼ����ȷ����ʼֵ����0 
    for (int i = 0; i < 3; ++i) {
        for (int j = 0; j < 3; ++j) { 
            B[i][j] = 0.0f;
        }
    }

    /* ����û��������ʲô���ҿ���������һ��ѭ����idx��0ѭ��n�Σ� 
    	Ȼ��ͬ������ͬ��ֵ
		���w����NULL����ִ��weights[idx]������ִ��w=1/n��������ʲô��˼��
		Ȼ����b��xyz����r��xyz���Ĳ�ͬ��ֵ����ֵ�ұ��ǽṹ��ĳ�Ա������body��ref��ʲô����
		Ȼ���Ƕ�B����Ĺ��죬����B������ʲô��������ô����ģ� 
	*/
    for (int idx = 0; idx < n; idx = idx+1) {
        float w = weights ? weights[idx] : 1.0f / n;
        float bx = body[idx].x;
        float by = body[idx].y;
        float bz = body[idx].z;
        float rx = ref[idx].x;
        float ry = ref[idx].y;
        float rz = ref[idx].z;

        B[0][0] += w * bx * rx;
        B[0][1] += w * bx * ry;
        B[0][2] += w * bx * rz;
        B[1][0] += w * by * rx;
        B[1][1] += w * by * ry;
        B[1][2] += w * by * rz;
        B[2][0] += w * bz * rx;
        B[2][1] += w * bz * ry;
        B[2][2] += w * bz * rz;
    }
}

/**
 * �����������ڹ���k���󣬵����Ҷ�k���󲢲���� 
 */
static void construct_K_matrix(float K[4][4], const float B[3][3]) {
    // S = B + B^T
    float S[3][3];
    for (int i = 0; i < 3; ++i) {
        for (int j = 0; j < 3; ++j) {
            S[i][j] = B[i][j] + B[j][i];
        }
    }

    float sigma = mat_trace(B);

    // Z ����
    float Z[3] = {
        B[1][2] - B[2][1],
        B[2][0] - B[0][2],
        B[0][1] - B[1][0]
    };

    // ��� K
    for (int i = 0; i < 3; ++i) {
        for (int j = 0; j < 3; ++j) {
            K[i][j] = S[i][j];
            if (i == j) K[i][j] -= sigma;
        }
        K[i][3] = Z[i];
        K[3][i] = Z[i];
    }
    K[3][3] = sigma;
}

/*����ai���ҵĽ��ͣ�������Ҳû�п��� 
  @brief �� 4x4 �Գƾ������ Jacobi �����ֽ⣨��ȡ�������ֵ��Ӧ������������
  
  @param A ����Գƾ��󣨻ᱻ�޸ģ�
  @param max_eigenvalue ����������ֵ
  @param eigenvector �����Ӧ����������������Ϊ 4��
  @param max_iter ����������
  @param tol ������ֵ
  @return true �ɹ���false δ����
 */
static bool jacobi_eigen_4x4(float A[4][4],
                             float* max_eigenvalue,
                             float eigenvector[4],
                             int max_iter,
                             float tol) {
    // ��ʼ����������Ϊ��λ����
    float V[4][4];
    for (int i = 0; i < 4; ++i) {
        for (int j = 0; j < 4; ++j) {
            V[i][j] = (i == j) ? 1.0f : 0.0f;
        }
    }

    // ��������
    float work[4][4];
    memcpy(work, A, sizeof(work));

    for (int iter = 0; iter < max_iter; ++iter) {
        // Ѱ�����ǶԽ�Ԫ
        float max_val = 0.0f;
        int p = 0, q = 0;
        for (int i = 0; i < 4; ++i) {
            for (int j = i + 1; j < 4; ++j) {
                float abs_val = fabsf(work[i][j]);
                if (abs_val > max_val) {
                    max_val = abs_val;
                    p = i;
                    q = j;
                }
            }
        }

        if (max_val < tol) {
            break;  // ����
        }

        // ������ת��
        float theta = 0.5f * atan2f(2.0f * work[p][q], work[q][q] - work[p][p]);
        float c = cosf(theta);
        float s = sinf(theta);

        // ���� work ����
        float App = work[p][p];
        float Aqq = work[q][q];
        float Apq = work[p][q];

        work[p][p] = c * c * App + s * s * Aqq - 2.0f * s * c * Apq;
        work[q][q] = s * s * App + c * c * Aqq + 2.0f * s * c * Apq;
        work[p][q] = work[q][p] = 0.0f;

        for (int r = 0; r < 4; ++r) {
            if (r != p && r != q) {
                float Apr = work[p][r];
                float Aqr = work[q][r];
                work[p][r] = c * Apr - s * Aqr;
                work[r][p] = work[p][r];
                work[q][r] = s * Apr + c * Aqr;
                work[r][q] = work[q][r];
            }
        }

        // ���������������� V = V * R
        for (int r = 0; r < 4; ++r) {
            float vrp = V[r][p];
            float vrq = V[r][q];
            V[r][p] = c * vrp - s * vrq;
            V[r][q] = s * vrp + c * vrq;
        }
    }

// ���￴�������ڽ����Խ���Ԫ���ḳֵ��max_val���棬�����Ϊ0��ֵ�����Ϊ�����ø�ֵ��������ʵû����ΪʲôҪ��һ��
//�жϣ��о��е���һ�٣� 
    int max_idx = 0;
    float max_val = work[0][0];
    for (int i = 1; i < 4; ++i) {
        if (work[i][i] > max_val) {
            max_val = work[i][i];
            max_idx = i;
        }
    }

    *max_eigenvalue = max_val;
    for (int i = 0; i < 4; ++i) {
        eigenvector[i] = V[i][max_idx];
    }

    /* �򵥼���Ƿ������������ǶԽ�Ԫ�Ժܴ󣬷��ؼ�
     ��������Ĭ���Ѿ�������ɣ������棬ʵ��Ӧ���пɸ�����Ҫ�жϣ�û����������ͣ�*/
    return true;
}

/**
 * @brief �����Ȩ���������
 */
static float compute_error(const DavenportVector3 body[],
                           const DavenportVector3 ref[],
                           const float weights[],
                           int n,
                           const DavenportQuaternion* q) {
    // �Ƚ���Ԫ��תΪ��ת����������
    float R[3][3];
    float w = q->w, x = q->x, y = q->y, z = q->z;
    R[0][0] = 1.0f - 2.0f * (y * y + z * z);
    R[0][1] = 2.0f * (x * y - w * z);
    R[0][2] = 2.0f * (x * z + w * y);
    R[1][0] = 2.0f * (x * y + w * z);
    R[1][1] = 1.0f - 2.0f * (x * x + z * z);
    R[1][2] = 2.0f * (y * z - w * x);
    R[2][0] = 2.0f * (x * z - w * y);
    R[2][1] = 2.0f * (y * z + w * x);
    R[2][2] = 1.0f - 2.0f * (x * x + y * y);

    float err = 0.0f;
    for (int i = 0; i < n; ++i) {
        float wgt = weights ? weights[i] : 1.0f / n;
        // Ԥ�Ȿ��ϵ����: R * ref[i]
        float px = R[0][0] * ref[i].x + R[0][1] * ref[i].y + R[0][2] * ref[i].z;
        float py = R[1][0] * ref[i].x + R[1][1] * ref[i].y + R[1][2] * ref[i].z;
        float pz = R[2][0] * ref[i].x + R[2][1] * ref[i].y + R[2][2] * ref[i].z;

        float dx = body[i].x - px;
        float dy = body[i].y - py;
        float dz = body[i].z - pz;

        err += wgt * (dx * dx + dy * dy + dz * dz);
    }
    return sqrtf(err / n);
}

/* ========================= ���� API ʵ�� ========================= */

void davenport_config_default(DavenportConfig* config) {
    if (config) {
        config->max_iterations = DAVENPORT_ITER_MAX;
        config->convergence_thresh = DAVENPORT_EPSILON;
    }
}

DavenportStatus davenport_solve(const DavenportConfig* config,
                        const DavenportVector3 body_vectors[],
                        const DavenportVector3 ref_vectors[],
                        const float weights[],
                        int num_vectors,
                        DavenportResult* result) {
    // �������
    if (!config || !body_vectors || !ref_vectors || !result) {
        return DAVENPORT_ERROR_INVALID_PARAM;
    }
    if (num_vectors < 2 || num_vectors > DAVENPORT_MAX_STARS) {
        return DAVENPORT_ERROR_NOT_ENOUGH_STARS;
    }

    // ������
    memset(result, 0, sizeof(DavenportResult));

    // ���� B ����
    float B[3][3];
    compute_B_matrix(B, body_vectors, ref_vectors, weights, num_vectors);

    // ���� K ����
    float K[4][4];
    construct_K_matrix(K, B);

    // �����ֽ����������ֵ����Ӧ��������
    float eigenvals[4];
    float eigenvec[4];
    bool converged = jacobi_eigen_4x4(K, &eigenvals[0], eigenvec,
                                      config->max_iterations,
                                      config->convergence_thresh);
    if (!converged) {
        return DAVENPORT_ERROR_NO_CONVERGENCE;
    }

    // ����������Ϊ��Ԫ����δ��һ����
    DavenportQuaternion q;
    q.x = eigenvec[0];
    q.y = eigenvec[1];
    q.z = eigenvec[2];
    q.w = eigenvec[3];

    // ��һ����Ԫ��
    float norm = sqrtf(q.w * q.w + q.x * q.x + q.y * q.y + q.z * q.z);
    if (norm < DAVENPORT_EPSILON) {
        return DAVENPORT_ERROR_SINGULAR;  // ���죬�޷���һ��
    }
    q.w /= norm;
    q.x /= norm;
    q.y /= norm;
    q.z /= norm;

    // ȷ���������ַǸ�����ѡ��
    if (q.w < 0) {
        q.w = -q.w;
        q.x = -q.x;
        q.y = -q.y;
        q.z = -q.z;
    }

    // �������
    float err = compute_error(body_vectors, ref_vectors, weights, num_vectors, &q);

    // �����
    result->quat = q;
    result->error_estimate = err;
    result->num_stars_used = num_vectors;

    return DAVENPORT_SUCCESS;
}

void davenport_quaternion_to_matrix(const DavenportQuaternion* quat, float mat[9]) {
    if (!quat || !mat) return;
    float w = quat->w, x = quat->x, y = quat->y, z = quat->z;
    mat[0] = 1.0f - 2.0f * (y * y + z * z);
    mat[1] = 2.0f * (x * y - w * z);
    mat[2] = 2.0f * (x * z + w * y);
    mat[3] = 2.0f * (x * y + w * z);
    mat[4] = 1.0f - 2.0f * (x * x + z * z);
    mat[5] = 2.0f * (y * z - w * x);
    mat[6] = 2.0f * (x * z - w * y);
    mat[7] = 2.0f * (y * z + w * x);
    mat[8] = 1.0f - 2.0f * (x * x + y * y);
}

void davenport_quaternion_to_euler(const DavenportQuaternion* quat,
                               float* roll, float* pitch, float* yaw) {
    if (!quat) return;
    float w = quat->w, x = quat->x, y = quat->y, z = quat->z;

    // ��ת����Ԫ�أ�������
    float r00 = 1.0f - 2.0f * (y * y + z * z);
    float r10 = 2.0f * (x * y + w * z);
    float r20 = 2.0f * (x * z - w * y);
    float r21 = 2.0f * (y * z + w * x);
    float r22 = 1.0f - 2.0f * (x * x + y * y);

    float p, r, yw;

    // ���������
    if (fabsf(r20) > SINGULARITY_THRESHOLD) {
        // ������Ϊ ��90��
        yw = 0.0f;  // Լ�� yaw = 0
        if (r20 < 0.0f) {
            p = 90.0f;
            r = atan2f(r10, r00) * 180.0f / (float)M_PI;
        } else {
            p = -90.0f;
            r = atan2f(-r10, -r00) * 180.0f / (float)M_PI;
        }
    } else {
        p = -asinf(r20) * 180.0f / (float)M_PI;
        float cp = cosf(p * (float)M_PI / 180.0f);
        r = atan2f(r21 / cp, r22 / cp) * 180.0f / (float)M_PI;
        yw = atan2f(r10 / cp, r00 / cp) * 180.0f / (float)M_PI;
    }

    if (roll) *roll = r;
    if (pitch) *pitch = p;
    if (yaw) *yaw = yw;
}
