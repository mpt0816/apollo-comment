/******************************************************************************
 * Copyright 2017 The Apollo Authors. All Rights Reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *****************************************************************************/

/**
 * @file: polynomial_xd.h
 **/

#pragma once

#include <cinttypes>
#include <vector>

namespace apollo {
namespace planning {

class PolynomialXd {
 public:
  PolynomialXd() = default;
  // order: 多项式的阶次
  explicit PolynomialXd(const std::uint32_t order);
  explicit PolynomialXd(const std::vector<double>& params);
  // 计算自变量为value时的多项式的值
  double operator()(const double value) const;
  // 返回自变量阶次为index的系数
  double operator[](const std::uint32_t index) const;
  void SetParams(const std::vector<double>& params);
  // 多项式求微分
  static PolynomialXd DerivedFrom(const PolynomialXd& base);
  // 多项式求积分
  static PolynomialXd IntegratedFrom(const PolynomialXd& base,
                                     const double intercept = 0.0);

  std::uint32_t order() const;
  const std::vector<double>& params() const;

 private:
  std::vector<double> params_;  // 多项式的系数，阶次高的系数排在后面
};

}  // namespace planning
}  // namespace apollo
