#pragma once
#include <array>
constexpr int N = 7; // 输入模糊子集的数量
constexpr int M = 2; // 同时隶属的模糊子集的数量
constexpr float NB = -1;
constexpr float NM = -0.66;
constexpr float NS = -0.33;
constexpr float ZO = 0;
constexpr float PS = 0.33;
constexpr float PM = 0.66;
constexpr float PB = 1;
// 比例增益
const std::array<std::array<float, N>, N> kp_rule_matrix =
    {{{PB, PB, PM, PM, PS, ZO, ZO},
      {PB, PB, PM, PS, PS, ZO, NS},
      {PM, PM, PM, PS, ZO, NS, NS},
      {PM, PM, PS, ZO, NS, NM, NM},
      {PS, PS, ZO, NS, NS, NM, NM},
      {PS, ZO, NS, NM, NM, NM, NB},
      {ZO, ZO, NM, NM, NM, NB, NB}}};
// 积分增益
const std::array<std::array<float, N>, N> ki_rule_matrix =
    {{{NB, NB, NM, NM, NS, ZO, ZO},
      {NB, NB, NM, NS, NS, ZO, ZO},
      {NB, NM, NS, NS, ZO, PS, PS},
      {NM, NM, NS, ZO, PS, PM, PM},
      {NM, NS, ZO, PS, PS, PM, PB},
      {ZO, ZO, PS, PS, PM, PB, PB},
      {ZO, ZO, PS, PM, PM, PB, PB}}};
// 微分增益
std::array<std::array<float, N>, N> kd_rule_matrix =
    {{{PS, NS, NB, NB, NB, NM, PS},
      {PS, NS, NB, NM, NM, NS, ZO},
      {ZO, NS, NM, NM, NS, NS, ZO},
      {ZO, NS, NS, NS, NS, NS, ZO},
      {ZO, ZO, ZO, ZO, ZO, ZO, ZO},
      {PB, NS, PS, PS, PS, PS, PB},
      {PB, PM, PM, PM, PS, PS, PB}}};

// 模糊论域上e和e_dot的取值
const std::array<float, N> e_range = {-1, -0.5, -0.25, 0, 0.25, 0.5, 1};  // 误差范围
const std::array<float, N> de_range = {-1, -0.5, -0.25, 0, 0.25, 0.5, 1}; // 误差导数范围