/*
 * =====================================================================
 *  多岔路口交通管理 —— 五岔路口交通灯设置问题
 *  （严蔚敏《数据结构(C语言版)》经典例题）
 * ---------------------------------------------------------------------
 *  问题描述：
 *      五岔路口中 C、E 为单行道（C 只出不进、E 只进不出），共有 13 条
 *      可通行路线。有的路线不能同时通行（如 E->B 与 A->D），有的可以
 *      同时通行（如 A->B 与 E->C）。要求设置尽可能多种颜色的交通灯，
 *      使冲突路线不同色、不冲突路线尽量同色（同一绿灯相位同时放行），
 *      以保证安全并使车流量最大。
 *
 *  数学建模（图着色模型）：
 *      顶点  = 13 条通行路线；
 *      边    = 两条路线在路口内的行驶轨迹交叉/汇合，不能同时放行；
 *      颜色  = 一种交通灯颜色（一个信号相位，同相位的路线同时绿灯）。
 *      问题转化为：对冲突图 G 做顶点着色，使相邻顶点颜色不同且
 *      颜色总数最少 —— 即求图 G 的色数 chi(G)。
 *
 *  数据结构定义：
 *      1) 路线表 routeNames[13]：顶点的一一对应命名（如 "AB"）；
 *      2) 冲突邻接矩阵 conflict[13][13]（int 型 0/1）：
 *         conflict[i][j]=1 当且仅当路线 i 与路线 j 在路口内冲突。
 *         该矩阵由路口几何结构（各路线轨迹是否相交）确定，是对称矩阵；
 *      3) color[13]：着色结果，color[i]=k 表示路线 i 使用第 k 种颜色。
 *
 *  核心算法：
 *      1) Welsh-Powell 贪心着色（求方案）：
 *         顶点按度数降序排列，依次对未着色顶点赋予新颜色，并尽可能多
 *         地把“与该颜色所有顶点均不相邻”的未着色顶点染成同色。
 *         时间复杂度：排序 O(V log V) + 着色 O(V^2)，V=13。
 *      2) 回溯法（验证最优性）：
 *         从 m=2 起逐一枚举 m 色可行方案（深度优先 + 约束剪枝：
 *         顶点只染“与已着色相邻顶点不冲突”的颜色），m=3 不可行而
 *         m=4 可行即证明最少灯色数为 4（冲突图中 {AC,BD,DA,EB}
 *         构成 K4 完全子图，理论上界亦为 4）。
 *         时间复杂度：最坏 O(m^V)，V=13、m<=4 时经剪枝毫秒级完成。
 *
 *  编译：gcc traffic_light.c -o traffic_light
 *  运行：./traffic_light
 * =====================================================================
 */

#include <stdio.h>
#include <string.h>
#include <ctype.h>

#define V 13                 /* 顶点数：13 条通行路线 */

/* 13 条通行路线（C 只出不进、E 只进不出，故无 C 开头、E 结尾的路线） */
static const char *routeNames[V] = {
    "AB", "AC", "AD", "BA", "BC", "BD", "DA",
    "DB", "DC", "EA", "EB", "EC", "ED"
};

/*
 * 冲突邻接矩阵：由路口几何结构确定（两条路线轨迹在路口内交叉或
 * 汇合即为 1）。对称矩阵，共 20 条冲突边。
 * 验证示例：E->B(10) 与 A->D(2) 冲突 conflict[10][2]=1；
 *           A->B(0) 与 E->C(11) 不冲突 conflict[0][11]=0。
 */
static const int conflict[V][V] = {
    /*AB AC AD BA BC BD DA DB DC EA EB EC ED */
    { 0, 0, 0, 0, 1, 1, 1, 0, 0, 1, 0, 0, 0},   /* AB */
    { 0, 0, 0, 0, 0, 1, 1, 1, 0, 1, 1, 0, 0},   /* AC */
    { 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 0},   /* AD */
    { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},   /* BA 无冲突 */
    { 1, 0, 0, 0, 0, 0, 0, 1, 0, 0, 1, 0, 0},   /* BC */
    { 1, 1, 0, 0, 0, 0, 1, 0, 0, 0, 1, 1, 0},   /* BD */
    { 1, 1, 0, 0, 0, 1, 0, 0, 0, 0, 1, 1, 0},   /* DA */
    { 0, 1, 0, 0, 1, 0, 0, 0, 0, 0, 0, 1, 0},   /* DB */
    { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},   /* DC 无冲突 */
    { 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},   /* EA */
    { 0, 1, 1, 0, 1, 1, 1, 0, 0, 0, 0, 0, 0},   /* EB */
    { 0, 0, 1, 0, 0, 1, 1, 1, 0, 0, 0, 0, 0},   /* EC */
    { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}    /* ED 无冲突 */
};

/* ------------------------- 基本工具函数 ------------------------- */

/* 计算顶点度数 */
static int degreeOf(int v)
{
    int d = 0, i;
    for (i = 0; i < V; i++) d += conflict[v][i];
    return d;
}

/*
 * 路线名查顶点编号：如 "AB" -> 0。
 * 校验规则：必须且仅为两个字母（自动转大写）、两字母不相同、
 * 且属于 13 条合法路线之一；非法输入返回 -1。
 */
static int findRoute(const char *s)
{
    char buf[8];
    size_t len = strlen(s);
    int i;
    char a, b;

    if (len == 0 || len > 2) return -1;      /* 长度校验 */
    for (i = 0; i < (int)len; i++) {
        if (!isalpha((unsigned char)s[i])) return -1;  /* 必须是字母 */
        buf[i] = (char)toupper((unsigned char)s[i]);
    }
    buf[len] = '\0';
    a = buf[0];
    b = (len == 2) ? buf[1] : '\0';

    if (a < 'A' || a > 'E') return -1;       /* 路口仅有 A~E 五个方向 */
    if (len == 2 && (b < 'A' || b > 'E')) return -1;
    if (len == 1 || a == b) return -1;       /* 起点终点不能相同 */

    for (i = 0; i < V; i++)
        if (strcmp(routeNames[i], buf) == 0) return i;
    return -1;                               /* 不在 13 条合法路线中 */
}

/* 启动自检：矩阵须对称、对角线为 0；返回冲突边数，-1 表示矩阵非法 */
static int selfCheck(void)
{
    int i, j, edges = 0;
    for (i = 0; i < V; i++) {
        if (conflict[i][i] != 0) return -1;          /* 对角线必须为 0 */
        for (j = i + 1; j < V; j++) {
            if (conflict[i][j] != conflict[j][i]) return -1; /* 必须对称 */
            edges += conflict[i][j];
        }
    }
    return edges;
}

/* 校验一种着色是否合法（相邻顶点不同色），用于结果自检 */
static int isColoringValid(const int color[])
{
    int i, j;
    for (i = 0; i < V; i++)
        for (j = i + 1; j < V; j++)
            if (conflict[i][j] && color[i] == color[j]) return 0;
    return 1;
}

/* ------------------------- 算法1：Welsh-Powell 贪心着色 ------------------------- */

/*
 * Welsh-Powell 算法：按度数降序（稳定次序）贪心着色。
 * color[] 输出各顶点颜色（0 起），返回值即使用的颜色总数。
 * 时间复杂度：O(V^2)（排序 O(V log V) 可忽略）。
 */
static int welshPowell(int color[])
{
    int order[V], i, j, used = 0;

    for (i = 0; i < V; i++) { order[i] = i; color[i] = -1; }

    /* 按度数降序排序（插入排序，稳定） */
    for (i = 1; i < V; i++) {
        int key = order[i];
        j = i - 1;
        while (j >= 0 && degreeOf(order[j]) < degreeOf(key)) {
            order[j + 1] = order[j];
            j--;
        }
        order[j + 1] = key;
    }

    for (i = 0; i < V; i++) {
        int v = order[i];
        if (color[v] != -1) continue;
        /* 对未着色的度数最大顶点启用一种新颜色 */
        color[v] = used;
        /* 尽可能多地吸收与该颜色所有顶点均不相邻的顶点 */
        for (j = i + 1; j < V; j++) {
            int u = order[j], k, ok = 1;
            if (color[u] != -1) continue;
            for (k = 0; k < V; k++)
                if (color[k] == used && conflict[u][k]) { ok = 0; break; }
            if (ok) color[u] = used;
        }
        used++;
    }
    return used;
}

/* ------------------------- 算法2：回溯法验证最优性 ------------------------- */

static int btColor[V];

/* 深度优先：为顶点 v 染色，共 m 色；成功返回 1 */
static int dfsColor(int v, int m)
{
    int c, u, ok;
    if (v == V) return 1;                    /* 全部着色成功 */
    for (c = 0; c < m; c++) {
        ok = 1;
        for (u = 0; u < V; u++)              /* 约束：与相邻顶点不同色 */
            if (conflict[v][u] && btColor[u] == c) { ok = 0; break; }
        if (!ok) continue;                   /* 剪枝 */
        btColor[v] = c;
        if (dfsColor(v + 1, m)) return 1;
        btColor[v] = -1;                     /* 回溯 */
    }
    return 0;
}

/* 判断图是否能用 m 种颜色着色（0/1）。最坏 O(m^V)，V=13 时毫秒级 */
static int canColorWith(int m)
{
    int i;
    for (i = 0; i < V; i++) btColor[i] = -1;
    return dfsColor(0, m);
}

/* ------------------------- 交互功能 ------------------------- */

static void printRoutes(void)
{
    int i;
    printf("\n===== 13 条通行路线（C 只出不进，E 只进不出） =====\n");
    for (i = 0; i < V; i++)
        printf("  路线%2d: %s   (冲突度: %d%s)\n", i, routeNames[i],
               degreeOf(i), degreeOf(i) == 0 ? "  <-- 无冲突路线，任意相位均可通行" : "");
}

static void printMatrix(void)
{
    int i, j;
    printf("\n===== 冲突邻接矩阵（1=冲突，0=可并行） =====\n");
    printf("     ");
    for (j = 0; j < V; j++) printf("%3s", routeNames[j]);
    printf("\n");
    for (i = 0; i < V; i++) {
        printf("%3s  ", routeNames[i]);
        for (j = 0; j < V; j++) printf("%3d", conflict[i][j]);
        printf("\n");
    }
}

static void genScheme(void)
{
    int color[V], used, k, i;
    used = welshPowell(color);
    printf("\n===== Welsh-Powell 贪心着色结果 =====\n");
    if (!isColoringValid(color)) {           /* 结果自检，防御性编程 */
        printf("错误：着色方案自检未通过！\n");
        return;
    }
    for (k = 0; k < used; k++) {
        printf("  相位 %d（灯色 %d）：", k + 1, k + 1);
        for (i = 0; i < V; i++)
            if (color[i] == k) printf("%s ", routeNames[i]);
        printf("\n");
    }
    printf("  共需 %d 种颜色（%d 个信号相位）\n", used, used);
}

static void verifyOptimal(void)
{
    int m = 2;
    printf("\n===== 回溯法验证最少灯色数 =====\n");
    while (m <= V) {
        if (canColorWith(m)) break;
        printf("  %d 种颜色：不可行（存在冲突无法避免）\n", m);
        m++;
    }
    printf("  %d 种颜色：可行\n", m);
    printf("  ==> 最少需要 %d 种颜色（理论下界：K4 子图 {AC,BD,DA,EB} 四边互冲）\n", m);
}

/* 读取一行并去掉尾部换行/空白；EOF 返回 0 */
static int readLine(char *buf, int size)
{
    int len;
    if (!fgets(buf, size, stdin)) return 0;
    len = (int)strlen(buf);
    while (len > 0 && (buf[len-1] == '\n' || buf[len-1] == '\r' ||
                       buf[len-1] == ' '  || buf[len-1] == '\t')) buf[--len] = '\0';
    return 1;
}

static void queryLoop(void)
{
    char buf[32];
    int r1, r2;
    printf("\n===== 路线兼容性查询（输入 Q 返回主菜单） =====\n");
    for (;;) {
        printf("请输入第一条路线（如 AB）: ");
        if (!readLine(buf, sizeof buf)) { printf("\n"); return; }   /* EOF 边界 */
        if (strcmp(buf, "Q") == 0 || strcmp(buf, "q") == 0) return;
        r1 = findRoute(buf);
        if (r1 < 0) { printf("  输入无效！须为 13 条合法路线之一（两字母 A~E、不同、非 C 开头、非 E 结尾）\n"); continue; }

        printf("请输入第二条路线（如 EC）: ");
        if (!readLine(buf, sizeof buf)) { printf("\n"); return; }
        if (strcmp(buf, "Q") == 0 || strcmp(buf, "q") == 0) return;
        r2 = findRoute(buf);
        if (r2 < 0) { printf("  输入无效！请重新输入。\n"); continue; }

        if (conflict[r1][r2])
            printf("  %s 与 %s 【冲突】：不能同时放行，须分配不同灯色\n", routeNames[r1], routeNames[r2]);
        else
            printf("  %s 与 %s 【兼容】：可分配同一灯色，同时绿灯放行\n", routeNames[r1], routeNames[r2]);
    }
}

int main(void)
{
    int edges = selfCheck(), choice;
    char buf[32];

    if (edges < 0) {
        printf("致命错误：冲突矩阵非法（须对称且对角线为 0）！\n");
        return 1;
    }
    printf("==================================================\n");
    printf("   多岔路口交通灯管理系统（图着色模型，V=13, E=%d）\n", edges);
    printf("   矩阵自检通过：对称、对角线为 0\n");
    printf("==================================================\n");

    for (;;) {
        printf("\n------------- 操作菜单 -----------\n");
        printf("  1. 查看 13 条路线及冲突矩阵\n");
        printf("  2. 生成交通灯设置方案（贪心着色）\n");
        printf("  3. 验证最少灯色数（回溯法）\n");
        printf("  4. 查询两条路线能否同时通行\n");
        printf("  0. 退出\n");
        printf("----------------------------------\n");
        printf("请选择: ");

        if (!readLine(buf, sizeof buf)) break;              /* EOF 安全退出 */
        if (sscanf(buf, "%d", &choice) != 1) {              /* 非数字输入校验 */
            printf("输入无效：请输入 0~4 的整数！\n");
            continue;
        }
        switch (choice) {
            case 1: printRoutes(); printMatrix(); break;
            case 2: genScheme(); break;
            case 3: verifyOptimal(); break;
            case 4: queryLoop(); break;
            case 0: printf("已退出。\n"); return 0;
            default: printf("输入越界：请选择 0~4！\n"); break;
        }
    }
    return 0;
}
