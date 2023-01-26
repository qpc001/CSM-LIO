/*
 * Copyright 2016 The Cartographer Authors
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *      http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#ifndef CSMLIO_MAPPING_3D_HYBRID_GRID_H_
#define CSMLIO_MAPPING_3D_HYBRID_GRID_H_

#include <array>
#include <cmath>
#include <limits>
#include <utility>
#include <vector>

#include "Eigen/Core"
#include "absl/memory/memory.h"
#include "csmlio/common/math.h"
#include "csmlio/common/port.h"
#include "csmlio/lio/probability_values.h"
#include "csmlio/lio/proto/hybrid_grid.pb.h"
#include "csmlio/transform/transform.h"
#include "glog/logging.h"

namespace csmlio {
namespace mapping {

// Converts an 'index' with each dimension from 0 to 2^'bits' - 1 to a flat
// z-major index.
inline int ToFlatIndex(const Eigen::Array3i& index, const int bits) {
  DCHECK((index >= 0).all() && (index < (1 << bits)).all()) << index;
  return (((index.z() << bits) + index.y()) << bits) + index.x();
}

// Converts a flat z-major 'index' to a 3-dimensional index with each dimension
// from 0 to 2^'bits' - 1.
inline Eigen::Array3i To3DIndex(const int index, const int bits) {
  DCHECK_LT(index, 1 << (3 * bits));
  const int mask = (1 << bits) - 1;
  return Eigen::Array3i(index & mask, (index >> bits) & mask,
                        (index >> bits) >> bits);
}

// A function to compare value to the default value. (Allows specializations).
template <typename TValueType>
bool IsDefaultValue(const TValueType& v) {
  return v == TValueType();
}

// Specialization to compare a std::vector to the default value.
template <typename TElementType>
bool IsDefaultValue(const std::vector<TElementType>& v) {
  return v.empty();
}

// A flat grid of '2^kBits' x '2^kBits' x '2^kBits' voxels storing values of
// type 'ValueType' in contiguous memory. Indices in each dimension are 0-based.
/**
 * @brief 3D地图结构的最底层数据，内部储存了('2^kBits' x '2^kBits' x '2^kBits')个网格的数据，对应3维
 * @tparam TValueType
 * @tparam kBits 表示每个维度上有(2^kBits)个网格(voxel)
 */
template <typename TValueType, int kBits>
class FlatGrid {
 public:
  using ValueType = TValueType;

  // Creates a new flat grid with all values being default constructed.
  /**
   * @brief 构造函数，初始化cells_数组
   * cells_数组大小为 2^{3kBits}
   */
  FlatGrid() {
    for (ValueType& value : cells_) {
      value = ValueType();
    }
  }

  FlatGrid(const FlatGrid&) = delete;
  FlatGrid& operator=(const FlatGrid&) = delete;

  // Returns the number of voxels per dimension.
  // 返回每个维度上的网格(voxel)数量
  static int grid_size() { return 1 << kBits; }

  // Returns the value stored at 'index', each dimension of 'index' being
  // between 0 and grid_size() - 1.
  // 输入3维索引，返回该索引在数组中对应的元素
  ValueType value(const Eigen::Array3i& index) const {
    return cells_[ToFlatIndex(index, kBits)];
  }

  // Returns a pointer to a value to allow changing it.
  // 输入3维索引，返回该索引在数组中对应的元素指针，便于修改
  ValueType* mutable_value(const Eigen::Array3i& index) {
    return &cells_[ToFlatIndex(index, kBits)];
  }

  // 内部实现了一个简单迭代器
  // An iterator for iterating over all values not comparing equal to the
  // default constructed value.
  class Iterator {
   public:
    Iterator() : current_(nullptr), end_(nullptr) {}

    explicit Iterator(const FlatGrid& flat_grid)
        : current_(flat_grid.cells_.data()),
          end_(flat_grid.cells_.data() + flat_grid.cells_.size()) {
      while (!Done() && IsDefaultValue(*current_)) {
        ++current_;
      }
    }

    void Next() {
      DCHECK(!Done());
      do {
        ++current_;
      } while (!Done() && IsDefaultValue(*current_));
    }

    bool Done() const { return current_ == end_; }

    Eigen::Array3i GetCellIndex() const {
      DCHECK(!Done());
      const int index = (1 << (3 * kBits)) - (end_ - current_);
      return To3DIndex(index, kBits);
    }

    const ValueType& GetValue() const {
      DCHECK(!Done());
      return *current_;
    }

   private:
    const ValueType* current_;
    const ValueType* end_;
  };

 private:
  std::array<ValueType, 1 << (3 * kBits)> cells_;
};

// A grid consisting of '2^kBits' x '2^kBits' x '2^kBits' grids of type
// 'WrappedGrid'. Wrapped grids are constructed on first access via
// 'mutable_value()'.
/**
 * @brief 中间数据层，内部包含了('2^kBits_2' x '2^kBits_2' x '2^kBits_2')个FlatGrid，
 * 也就是每个NestedGrid，内部包含了(2^kBits_1 x 2^kBits_2, 2^kBits_1 x 2^kBits_2， 2^kBits_1 x 2^kBits_2)个网格
 * @tparam WrappedGrid 传进来的是FlatGrid
 * @tparam kBits 就是@brief中的kBits_2(一般是3)
 */
template <typename WrappedGrid, int kBits>
class NestedGrid {
 public:
  // NestedGrid::ValueType = FlatGrid::ValueType = float(double) ?
  using ValueType = typename WrappedGrid::ValueType;

  // Returns the number of voxels per dimension.
  // 返回每个维度上的网格数量
  // 每个NestedGrid，内部包含了(2^kBits_1 x 2^kBits_2, 2^kBits_1 x 2^kBits_2， 2^kBits_1 x 2^kBits_2)个网格
  // 所以，grid_size = FlatGrid::grid_size() * 2^kBits_2
  //                = 2^kBits_1 x 2^kBits_2
  static int grid_size() { return WrappedGrid::grid_size() << kBits; }

  // Returns the value stored at 'index', each dimension of 'index' being
  // between 0 and grid_size() - 1.
  /**
   * @brief 输入在本NestedGrid内给定网格的索引idx，获取对应的网格：
   *        1. 先获取对应的FlatGrid索引
   *        2. 再获取指定网格在FlatGrid中的内部索引
   *        3. 从FlatGrid中获取网格
   * @param index
   * @return
   */
  ValueType value(const Eigen::Array3i& index) const {
    // 输入给定网格的索引idx，获取对应的FlatGrid的索引，称为meta_index
    const Eigen::Array3i meta_index = GetMetaIndex(index);
    // 尝试取对应的FlatGrid，
    // meta_cells_是一张二维的表格，储存着FlatGrid的指针
    // 当初始化时，meta_cells_表格还是空的，在下面的mutable_value()函数中会进行构造和初始化，也就是开辟空间
    const WrappedGrid* const meta_cell =
        meta_cells_[ToFlatIndex(meta_index, kBits)].get();
    // 如果还没初始化，内存空间还开辟，随便返回一个ValueType()
    if (meta_cell == nullptr) {
      return ValueType();
    }
    // 获取指定网格在FlatGrid中的索引，称为inner_index
    const Eigen::Array3i inner_index =
        index - meta_index * WrappedGrid::grid_size();
    // 从FlatGrid取对应的网格，即为所需的网格
    return meta_cell->value(inner_index);
  }

  // Returns a pointer to the value at 'index' to allow changing it. If
  // necessary a new wrapped grid is constructed to contain that value.
  /**
   * @brief 输入给定网格的索引idx，获取对应的网格指针：
   *        1. 先获取对应的FlatGrid索引
   *        2. 检查对应的FlatGrid是否存在（开辟内存空间），如果还没初始化，则创建一个
   *        4. 再获取指定网格在FlatGrid中的内部索引
   *        5. 从FlatGrid中获取网格指针
   * @param index
   * @return
   */
  ValueType* mutable_value(const Eigen::Array3i& index) {
    const Eigen::Array3i meta_index = GetMetaIndex(index);
    std::unique_ptr<WrappedGrid>& meta_cell =
        meta_cells_[ToFlatIndex(meta_index, kBits)];
    if (meta_cell == nullptr) {
      meta_cell = absl::make_unique<WrappedGrid>();
    }
    const Eigen::Array3i inner_index =
        index - meta_index * WrappedGrid::grid_size();
    return meta_cell->mutable_value(inner_index);
  }

  /**
   * @brief 内部实现的迭代器，注意是对底层网格的迭代
   */
  // An iterator for iterating over all values not comparing equal to the
  // default constructed value.
  class Iterator {
   public:
    Iterator() : current_(nullptr), end_(nullptr), nested_iterator_() {}

    explicit Iterator(const NestedGrid& nested_grid)
        : current_(nested_grid.meta_cells_.data()),
          end_(nested_grid.meta_cells_.data() + nested_grid.meta_cells_.size()),
          nested_iterator_() {
      AdvanceToValidNestedIterator();
    }

    void Next() {
      DCHECK(!Done());
      nested_iterator_.Next();
      if (!nested_iterator_.Done()) {
        return;
      }
      ++current_;
      AdvanceToValidNestedIterator();
    }

    bool Done() const { return current_ == end_; }

    Eigen::Array3i GetCellIndex() const {
      DCHECK(!Done());
      const int index = (1 << (3 * kBits)) - (end_ - current_);
      return To3DIndex(index, kBits) * WrappedGrid::grid_size() +
             nested_iterator_.GetCellIndex();
    }

    const ValueType& GetValue() const {
      DCHECK(!Done());
      return nested_iterator_.GetValue();
    }

   private:
    void AdvanceToValidNestedIterator() {
      for (; !Done(); ++current_) {
        if (*current_ != nullptr) {
          nested_iterator_ = typename WrappedGrid::Iterator(**current_);
          if (!nested_iterator_.Done()) {
            break;
          }
        }
      }
    }

    const std::unique_ptr<WrappedGrid>* current_;
    const std::unique_ptr<WrappedGrid>* end_;
    typename WrappedGrid::Iterator nested_iterator_;
  };

 private:
  // Returns the Eigen::Array3i (meta) index of the meta cell containing
  // 'index'.
  /**
   * @brief 输入给定网格的索引idx，获取对应的FlatGrid的索引，称为meta_index
   * @param index 给定网格的索引idx
   * @return
   */
  Eigen::Array3i GetMetaIndex(const Eigen::Array3i& index) const {
    DCHECK((index >= 0).all()) << index;
    // 获取给定网格对应的FlatGrid的索引，称为meta_index
    const Eigen::Array3i meta_index = index / WrappedGrid::grid_size();
    DCHECK((meta_index < (1 << kBits)).all()) << index;
    return meta_index;
  }

  std::array<std::unique_ptr<WrappedGrid>, 1 << (3 * kBits)> meta_cells_; ///< 表格，存放了2^(3kBits)个FlatGrid指针
};

// A grid consisting of 2x2x2 grids of type 'WrappedGrid' initially. Wrapped
// grids are constructed on first access via 'mutable_value()'. If necessary,
// the grid grows to twice the size in each dimension. The range of indices is
// (almost) symmetric around the origin, i.e. negative indices are allowed.
/**
 * @brief 数据结构层面上，3D地图的最上层，后面的实现只是继承这个类
 * 初始时，内部包含了(2 x 2 x 2)个NestedGrid，后续可扩展
 * 也就是每个DynamicGrid，内部包含了(2 x 2^kBits_1 x 2^kBits_2,            // 2 x 2^3 x 2^3 = 128
 *                               2 x 2^kBits_1 x 2^kBits_2,
 *                               2 x 2^kBits_1 x 2^kBits_2)个网格
 * @tparam WrappedGrid
 */
template <typename WrappedGrid>
class DynamicGrid {
 public:
  // DynamicGrid::ValueType = NestedGrid::ValueType = FlatGrid::ValueType = float(double) ?
  using ValueType = typename WrappedGrid::ValueType;

  /**
   * @brief 构造函数，直接指定了 bits_ = 1 （后续这个bits_随地图扩大而增长）
   */
  DynamicGrid() : bits_(1), meta_cells_(8) {}
  DynamicGrid(DynamicGrid&&) = default;
  DynamicGrid& operator=(DynamicGrid&&) = default;

  // Returns the current number of voxels per dimension.
  // 获取每个维度上的网格数量
  // 每个DynamicGrid，内部包含了(2 x 2 x 2)个NestedGrid
  // 所以，grid_size = NestedGrid::grid_size() * 2
  //                = 2^kBits_1 x 2^kBits_2 x 2
  int grid_size() const { return WrappedGrid::grid_size() << bits_; }

  // Returns the value stored at 'index'.
  /**
   * @brief 输入指定网格索引idx，获取概率地图中对应网格的值
   * @param index
   * @return
   */
  ValueType value(const Eigen::Array3i& index) const {
    // 输入指定网格索引idx,对输入的idx进行偏移，确保shifted_index均大于0
    const Eigen::Array3i shifted_index = index + (grid_size() >> 1);
    // The cast to unsigned is for performance to check with 3 comparisons
    // shifted_index.[xyz] >= 0 and shifted_index.[xyz] < grid_size.
    if ((shifted_index.cast<unsigned int>() >= grid_size()).any()) {
      return ValueType();
    }
    // 尝试取对应的NestedGrid，如果为空，则返回ValueType()
    const Eigen::Array3i meta_index = GetMetaIndex(shifted_index);
    const WrappedGrid* const meta_cell =
        meta_cells_[ToFlatIndex(meta_index, bits_)].get();
    if (meta_cell == nullptr) {
      return ValueType();
    }
    // 否则，则计算指定网格索引idx在NestedGrid中对应的索引inner_index
    const Eigen::Array3i inner_index =
        shifted_index - meta_index * WrappedGrid::grid_size();
    // 进一步调用NestedGrid::value
    return meta_cell->value(inner_index);
  }

  // Returns a pointer to the value at 'index' to allow changing it, dynamically
  // growing the DynamicGrid and constructing new WrappedGrids as needed.
  ValueType* mutable_value(const Eigen::Array3i& index) {
    const Eigen::Array3i shifted_index = index + (grid_size() >> 1);
    // The cast to unsigned is for performance to check with 3 comparisons
    // shifted_index.[xyz] >= 0 and shifted_index.[xyz] < grid_size.
    if ((shifted_index.cast<unsigned int>() >= grid_size()).any()) {
      Grow();
      return mutable_value(index);
    }
    // 尝试取对应的NestedGrid，
    // meta_cells_是一张二维的表格，储存着NestedGrid的指针
    // 当初始化时，meta_cells_表格还是空的，在下面的mutable_value()函数中会进行构造和初始化，也就是开辟空间
    const Eigen::Array3i meta_index = GetMetaIndex(shifted_index);
    std::unique_ptr<WrappedGrid>& meta_cell =
        meta_cells_[ToFlatIndex(meta_index, bits_)];
    if (meta_cell == nullptr) {
      meta_cell = absl::make_unique<WrappedGrid>();
    }
    const Eigen::Array3i inner_index =
        shifted_index - meta_index * WrappedGrid::grid_size();
    return meta_cell->mutable_value(inner_index);
  }

  // An iterator for iterating over all values not comparing equal to the
  // default constructed value.
  class Iterator {
   public:
    explicit Iterator(const DynamicGrid& dynamic_grid)
        : bits_(dynamic_grid.bits_),
          current_(dynamic_grid.meta_cells_.data()),
          end_(dynamic_grid.meta_cells_.data() +
               dynamic_grid.meta_cells_.size()),
          nested_iterator_() {
      AdvanceToValidNestedIterator();
    }

    void Next() {
      DCHECK(!Done());
      nested_iterator_.Next();
      if (!nested_iterator_.Done()) {
        return;
      }
      ++current_;
      AdvanceToValidNestedIterator();
    }

    bool Done() const { return current_ == end_; }

    Eigen::Array3i GetCellIndex() const {
      DCHECK(!Done());
      const int outer_index = (1 << (3 * bits_)) - (end_ - current_);
      const Eigen::Array3i shifted_index =
          To3DIndex(outer_index, bits_) * WrappedGrid::grid_size() +
          nested_iterator_.GetCellIndex();
      return shifted_index - ((1 << (bits_ - 1)) * WrappedGrid::grid_size());
    }

    const ValueType& GetValue() const {
      DCHECK(!Done());
      return nested_iterator_.GetValue();
    }

    void AdvanceToEnd() { current_ = end_; }

    const std::pair<Eigen::Array3i, ValueType> operator*() const {
      return std::pair<Eigen::Array3i, ValueType>(GetCellIndex(), GetValue());
    }

    Iterator& operator++() {
      Next();
      return *this;
    }

    bool operator!=(const Iterator& it) const {
      return it.current_ != current_;
    }

   private:
    void AdvanceToValidNestedIterator() {
      for (; !Done(); ++current_) {
        if (*current_ != nullptr) {
          nested_iterator_ = typename WrappedGrid::Iterator(**current_);
          if (!nested_iterator_.Done()) {
            break;
          }
        }
      }
    }

    int bits_;
    const std::unique_ptr<WrappedGrid>* current_;
    const std::unique_ptr<WrappedGrid>* const end_;
    typename WrappedGrid::Iterator nested_iterator_;
  };

 private:
  // Returns the Eigen::Array3i (meta) index of the meta cell containing
  // 'index'.
  Eigen::Array3i GetMetaIndex(const Eigen::Array3i& index) const {
    DCHECK((index >= 0).all()) << index;
    // 获取给定网格对应的NestedGrid的索引，称为meta_index
    const Eigen::Array3i meta_index = index / WrappedGrid::grid_size();
    DCHECK((meta_index < (1 << bits_)).all()) << index;
    return meta_index;
  }

  // Grows this grid by a factor of 2 in each of the 3 dimensions.
  /**
   * @brief 对当前地图进行扩展，每个维度上扩展为原来的两倍
   */
  void Grow() {
    const int new_bits = bits_ + 1;
    CHECK_LE(new_bits, 8);
    // 新创建一个std::vector<std::unique_ptr<WrappedGrid>>
    std::vector<std::unique_ptr<WrappedGrid>> new_meta_cells_(
        8 * meta_cells_.size());
    // move操作
    for (int z = 0; z != (1 << bits_); ++z) {
      for (int y = 0; y != (1 << bits_); ++y) {
        for (int x = 0; x != (1 << bits_); ++x) {
          const Eigen::Array3i original_meta_index(x, y, z);
          // 计算原来的索引在扩展后的地图中的索引
          const Eigen::Array3i new_meta_index =
              original_meta_index + (1 << (bits_ - 1));
          // move操作
          new_meta_cells_[ToFlatIndex(new_meta_index, new_bits)] =
              std::move(meta_cells_[ToFlatIndex(original_meta_index, bits_)]);
        }
      }
    }
    meta_cells_ = std::move(new_meta_cells_);
    bits_ = new_bits;
  }

  int bits_;
  std::vector<std::unique_ptr<WrappedGrid>> meta_cells_;
};

template <typename ValueType>
using GridBase = DynamicGrid<NestedGrid<FlatGrid<ValueType, 3>, 3>>;

// Represents a 3D grid as a wide, shallow tree.
/**
 * @brief 在DynamicGrid<NestedGrid<FlatGrid<ValueType, 3>, 3>>基础上，增加了resolution相关内容
 * @tparam ValueType
 */
template <typename ValueType>
class HybridGridBase : public GridBase<ValueType> {
 public:
  using Iterator = typename GridBase<ValueType>::Iterator;

  // Creates a new tree-based probability grid with voxels having edge length
  // 'resolution' around the origin which becomes the center of the cell at
  // index (0, 0, 0).
  /**
   * @brief 给定分辨率，创建概率网格
   * @param resolution
   */
  explicit HybridGridBase(const float resolution) : resolution_(resolution) {}

  float resolution() const { return resolution_; }

  // Returns the index of the cell containing the 'point'. Indices are integer
  // vectors identifying cells, for this the coordinates are rounded to the next
  // multiple of the resolution.
  /**
   * @brief 输入点坐标，获取对应的网格索引idx
   * @param point
   * @return
   */
  Eigen::Array3i GetCellIndex(const Eigen::Vector3f& point) const {
    Eigen::Array3f index = point.array() / resolution_;
    // std::lround四舍五入取整
    return Eigen::Array3i(common::RoundToInt(index.x()),
                          common::RoundToInt(index.y()),
                          common::RoundToInt(index.z()));
  }

  // Returns one of the octants, (0, 0, 0), (1, 0, 0), ..., (1, 1, 1).
  /**
   * @brief 输入[0~8)的整数，获取对应的网格
   * @param i
   * @return
   */
  static Eigen::Array3i GetOctant(const int i) {
    DCHECK_GE(i, 0);
    DCHECK_LT(i, 8);
    return Eigen::Array3i(static_cast<bool>(i & 1), static_cast<bool>(i & 2),
                          static_cast<bool>(i & 4));
  }

  // Returns the center of the cell at 'index'.
  /**
   * @brief 输入网格索引idx，返回对应网格的中心点
   * @param index
   * @return
   */
  Eigen::Vector3f GetCenterOfCell(const Eigen::Array3i& index) const {
    return index.matrix().cast<float>() * resolution_;
  }

  // Iterator functions for range-based for loops.
  Iterator begin() const { return Iterator(*this); }

  Iterator end() const {
    Iterator it(*this);
    it.AdvanceToEnd();
    return it;
  }

 private:
  // Edge length of each voxel.
  const float resolution_;
};

// A grid containing probability values stored using 15 bits, and an update
// marker per voxel.
// Points are expected to be close to the origin. Points far from the origin
// require the grid to grow dynamically. For centimeter resolution, points
// can only be tens of meters from the origin.
// The hard limit of cell indexes is +/- 8192 around the origin.
/**
 * @brief 3D的地图类型，最顶层
 *
 * template <typename ValueType>
 * using GridBase = DynamicGrid<NestedGrid<FlatGrid<ValueType, 3>, 3> >;
 *
 * HybridGridBase 继承 DynamicGrid<NestedGrid<FlatGrid<ValueType, 3>, 3>>
 * class HybridGridBase : public GridBase<ValueType>
 */
class HybridGrid : public HybridGridBase<uint16> {
 public:
  /**
   * @brief 构造函数，主要是初始化基类HybridGridBase<uint16>
   * @param resolution
   */
  explicit HybridGrid(const float resolution)
      : HybridGridBase<uint16>(resolution) {}

  /**
   * @brief 从文件中读取地图数据
   * @param proto
   */
  explicit HybridGrid(const proto::HybridGrid& proto)
      : HybridGrid(proto.resolution()) {
    CHECK_EQ(proto.values_size(), proto.x_indices_size());
    CHECK_EQ(proto.values_size(), proto.y_indices_size());
    CHECK_EQ(proto.values_size(), proto.z_indices_size());
    for (int i = 0; i < proto.values_size(); ++i) {
      // SetProbability does some error checking for us.
      SetProbability(Eigen::Vector3i(proto.x_indices(i), proto.y_indices(i),
                                     proto.z_indices(i)),
                     ValueToProbability(proto.values(i)));
    }
  }

  // Sets the probability of the cell at 'index' to the given 'probability'.
  /**
   * @brief 输入指定网格索引idx，以及对应的概率，计算概率对应的uint16整数，并赋值
   * @param index
   * @param probability
   */
  void SetProbability(const Eigen::Array3i& index, const float probability) {
    *mutable_value(index) = ProbabilityToValue(probability);
  }

  /**
   * @brief 完成空闲概率的更新(实际上就是减去查表的时候,表格中所附带的一个偏移值kUpdateMarker)
   */
  // Finishes the update sequence.
  void FinishUpdate() {
    while (!update_indices_.empty()) {
      DCHECK_GE(*update_indices_.back(), kUpdateMarker);
      *update_indices_.back() -= kUpdateMarker;
      update_indices_.pop_back();
    }
  }

  // Applies the 'odds' specified when calling ComputeLookupTableToApplyOdds()
  // to the probability of the cell at 'index' if the cell has not already been
  // updated. Multiple updates of the same cell will be ignored until
  // FinishUpdate() is called. Returns true if the cell was updated.
  //
  // If this is the first call to ApplyOdds() for the specified cell, its value
  // will be set to probability corresponding to 'odds'.
  /**
   * @brief 通过查表来更新给定索引[x,y,z]网格单元的空闲概率
   * @param index
   * @param table
   * @return
   */
  bool ApplyLookupTable(const Eigen::Array3i& index,
                        const std::vector<uint16>& table) {
    DCHECK_EQ(table.size(), kUpdateMarker);
    // 取对应索引的网格指针uint16*
    uint16* const cell = mutable_value(index);
    // 如果网格的值大于阈值kUpdateMarker，则直接返回false
    if (*cell >= kUpdateMarker) {
      return false;
    }
    // 否则，在update_indices_队列中加入该cell网格指针
    update_indices_.push_back(cell);
    // 查表?
    *cell = table[*cell];
    DCHECK_GE(*cell, kUpdateMarker);
    return true;
  }

  // Returns the probability of the cell with 'index'.
  /**
   * @brief 获取指定索引网格的概率
   * @param index
   * @return
   */
  float GetProbability(const Eigen::Array3i& index) const {
    // 先调用value(index)取出uint16类型的值
    // 然后ValueToProbability()转换为float概率
    return ValueToProbability(value(index));
  }

  // Returns true if the probability at the specified 'index' is known.
  bool IsKnown(const Eigen::Array3i& index) const { return value(index) != 0; }

  proto::HybridGrid ToProto() const {
    CHECK(update_indices_.empty()) << "Serializing a grid during an update is "
                                      "not supported. Finish the update first.";
    proto::HybridGrid result;
    result.set_resolution(resolution());
    for (const auto it : *this) {
      result.add_x_indices(it.first.x());
      result.add_y_indices(it.first.y());
      result.add_z_indices(it.first.z());
      result.add_values(it.second);
    }
    return result;
  }

 private:
  // Markers at changed cells.
  std::vector<ValueType*> update_indices_;  ///< 记录更新过的栅格单元的存储索引
};

struct AverageIntensityData {
  float sum = 0.f;
  int count = 0;
};

class IntensityHybridGrid : public HybridGridBase<AverageIntensityData> {
 public:
  explicit IntensityHybridGrid(const float resolution)
      : HybridGridBase<AverageIntensityData>(resolution) {}

  void AddIntensity(const Eigen::Array3i& index, const float intensity) {
    AverageIntensityData* const cell = mutable_value(index);
    cell->count += 1;
    cell->sum += intensity;
  }

  float GetIntensity(const Eigen::Array3i& index) const {
    const AverageIntensityData& cell = value(index);
    if (cell.count == 0) {
      return 0.f;
    } else {
      return cell.sum / cell.count;
    }
  }
};

}  // namespace mapping
}  // namespace csmlio

#endif  // CSMLIO_MAPPING_3D_HYBRID_GRID_H_
