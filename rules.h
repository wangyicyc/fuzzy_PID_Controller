#pragma once
const int N = 7; // 输入模糊子集的数量
const int M = 2; // 同时隶属的模糊子集的数量
#define NB -1
#define NM -0.66
#define NS -0.33
#define ZO 0
#define PS 0.33
#define PM 0.66
#define PB 1

float kp_rule_matrix[N][N] = {{PB, PB, PM, PM, PS, ZO, ZO},
                            {PB, PB, PM, PS, PS, ZO, NS},
                            {PM, PM, PM, PS, ZO, NS, NS},
                            {PM, PM, PS, ZO, NS, NM, NM},
                            {PS, PS, ZO, NS, NS, NM, NM},
                            {PS, ZO, NS, NM, NM, NM, NB},
                            {ZO, ZO, NM, NM, NM, NB, NB}};
// 积分增益
float ki_rule_matrix[N][N] = {{NB, NB, NM, NM, NS, ZO, ZO},
                            {NB, NB, NM, NS, NS, ZO, ZO},
                            {NB, NM, NS, NS, ZO, PS, PS},
                            {NM, NM, NS, ZO, PS, PM, PM},
                            {NM, NS, ZO, PS, PS, PM, PB},
                            {ZO, ZO, PS, PS, PM, PB, PB},
                            {ZO, ZO, PS, PM, PM, PB, PB}};
// 微分增益
float kd_rule_matrix[N][N] = {{PS, NS, NB, NB, NB, NM, PS},
                            {PS, NS, NB, NM, NM, NS, ZO},
                            {ZO, NS, NM, NM, NS, NS, ZO},
                            {ZO, NS, NS, NS, NS, NS, ZO},
                            {ZO, ZO, ZO, ZO, ZO, ZO, ZO},
                            {PB, NS, PS, PS, PS, PS, PB},
                            {PB, PM, PM, PM, PS, PS, PB}};

// 模糊论域上e和e_dot的取值
float e_range[N] = {-1, -0.5, -0.25, 0, 0.25, 0.5, 1}; // 误差范围
float de_range[N] = {-1, -0.5, -0.25, 0, 0.25, 0.5, 1}; // 误差导数范围