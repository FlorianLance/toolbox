
/*******************************************************************************
** Toolbox-base                                                               **
** MIT License                                                                **
** Copyright (c) [2018] [Florian Lance]                                       **
**                                                                            **
** Permission is hereby granted, free of charge, to any person obtaining a    **
** copy of this software and associated documentation files (the "Software"), **
** to deal in the Software without restriction, including without limitation  **
** the rights to use, copy, modify, merge, publish, distribute, sublicense,   **
** and/or sell copies of the Software, and to permit persons to whom the      **
** Software is furnished to do so, subject to the following conditions:       **
**                                                                            **
** The above copyright notice and this permission notice shall be included in **
** all copies or substantial portions of the Software.                        **
**                                                                            **
** THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR **
** IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,   **
** FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL    **
** THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER **
** LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING    **
** FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER        **
** DEALINGS IN THE SOFTWARE.                                                  **
**                                                                            **
********************************************************************************/

#pragma once

// std
#include <tuple>
#include <stdexcept>
#include <algorithm>
#include <array>
#include <vector>
#include <iterator>
#include <optional>

namespace tool {

//    template <typename TupleT>
//    struct DynTupleArray {
//        int size;
//        TupleT *data;
//    };
//        static constexpr auto generate(DynTupleArray<TupleT> &a){
//            auto res = TupleArray{a.data, a.size};
//            delete[] a.data;
//            return res;
//        }

    template <auto Size, typename TupleT>
    struct TupleArray {

        std::array<TupleT, static_cast<size_t>(Size)> data;

        // first row element which tuple of column n°ColumnIdTypeToCompare equals input value
        template <std::size_t ColumnIdTypeToCompare>
        [[nodiscard]] constexpr TupleT at(const typename std::tuple_element<ColumnIdTypeToCompare, TupleT>::type &value) const {
            const auto itr = std::find_if(begin(data), end(data), [&value](const auto &v) {
                return std::get<ColumnIdTypeToCompare>(v) == value;
            });
            if (itr != end(data)) {
                return *itr;
            } else {
                return {}; // throw std::range_error("Not Found");
            }
        }

        // get column ColumnIdTypeToGet of first row element which tuple of column ColumnIdTypeToCompare equals input value
        template <std::size_t ColumnIdTypeToCompare, std::size_t ColumnIdTypeToGet>
        [[nodiscard]] constexpr typename std::tuple_element<ColumnIdTypeToGet, TupleT>::type at(const typename std::tuple_element<ColumnIdTypeToCompare, TupleT>::type &value) const {
            return std::get<ColumnIdTypeToGet>(at<ColumnIdTypeToCompare>(value));
        }

        // first row element which tuple of column ColumnIdTypeToCompare equals input value
        template <std::size_t ColumnIdTypeToCompare>
        [[nodiscard]] constexpr std::optional<TupleT> optional_at(const typename std::tuple_element<ColumnIdTypeToCompare, TupleT>::type &value) const {
             const auto itr = std::find_if(begin(data), end(data), [&value](const auto &v) {
                 return std::get<ColumnIdTypeToCompare>(v) == value;
             });
             if (itr != end(data)) {
                 return *itr;
             } else {
                 return {};
             }
        }

        // get column ColumnIdTypeToGet of first row element which tuple of column ColumnIdTypeToCompare equals input value
        template <std::size_t ColumnIdTypeToCompare, std::size_t ColumnIdTypeToGet>
        [[nodiscard]] constexpr std::optional<typename std::tuple_element<ColumnIdTypeToGet, TupleT>::type> optional_at(const typename std::tuple_element<ColumnIdTypeToCompare, TupleT>::type &value) const {
            const auto itr = std::find_if(begin(data), end(data), [&value](const auto &v) {
                return std::get<ColumnIdTypeToCompare>(v) == value;
            });
            if (itr != end(data)) {
                return std::get<ColumnIdTypeToGet>(*itr);
            } else {
                return {};
            }
        }

        // count elements with tuple of column n°idKey equals input key
        template <std::size_t ColumnId>
        [[nodiscard]] constexpr size_t count_equal(const typename std::tuple_element<ColumnId, TupleT>::type &columnValue) const {
            return std::count_if(std::begin(data), std::end(data), [&](const TupleT &v){
                return std::get<ColumnId>(v) == columnValue;
            });
        }


        template <std::size_t ColumnId>
        [[nodiscard]] constexpr size_t count_not_equal(const typename std::tuple_element<ColumnId, TupleT>::type &columnValue) const {
            return std::count_if(std::begin(data), std::end(data), [&](const TupleT &v){
                return std::get<ColumnId>(v) != columnValue;
            });
        }

        template <std::size_t IdTupleElement>
        [[nodiscard]] constexpr auto tuple_column() const{
            std::array<typename std::tuple_element<IdTupleElement, TupleT>::type, static_cast<size_t>(Size)> column = {};
            for(size_t ii = 0; ii < data.size(); ++ii){
                column[ii] = std::get<IdTupleElement>(data[ii]);
            }
            return column;
        }


//        template <std::size_t ColumnId>
//        constexpr DynTupleArray<TupleT> filter(const typename std::tuple_element<ColumnId, TupleT>::type &columnValue){
//            auto size = count_equal<ColumnId>(columnValue);
//            DynTupleArray<TupleT> a;
//            a.size = size;
//            a.data = new TupleT[size];

//            std::copy_if(std::begin(data), std::end(data), std::back_inserter(a.data), [&](const TupleT &v) {
//                return std::get<ColumnId>(v) == columnValue;
//            });
//            return a;
//        }




        // retrieve all lines tuples which column n°ColumnId equals their column value
        template <std::size_t ColumnId>
        [[nodiscard]] auto tuples_matching_columns_values(const typename std::tuple_element<ColumnId, TupleT>::type &columnValue) const {

            std::vector<TupleT> tuples;
            tuples.reserve(static_cast<size_t>(Size));
            std::copy_if(std::begin(data), std::end(data), std::back_inserter(tuples), [&](const TupleT &v) {
                return std::get<ColumnId>(v) == columnValue;
            });
            return tuples;
        }        

        template <std::size_t ColumnId1, std::size_t ColumnId2>
        [[nodiscard]] auto tuples_matching_columns_values(
            const typename std::tuple_element<ColumnId1, TupleT>::type &columnValue1,
            const typename std::tuple_element<ColumnId2, TupleT>::type &columnValue2) const {

            std::vector<TupleT> tuples;
            tuples.reserve(static_cast<size_t>(Size));
            std::copy_if(std::begin(data), std::end(data), std::back_inserter(tuples), [&](const TupleT &v) {
                return (std::get<ColumnId1>(v) == columnValue1) && (std::get<ColumnId2>(v) == columnValue2);
            });
            return tuples;
        }

        template <std::size_t ColumnId1, std::size_t ColumnId2, std::size_t ColumnId3>
        [[nodiscard]] auto tuples_matching_columns_values(
            const typename std::tuple_element<ColumnId1, TupleT>::type &columnValue1,
            const typename std::tuple_element<ColumnId2, TupleT>::type &columnValue2,
            const typename std::tuple_element<ColumnId3, TupleT>::type &columnValue3) const {

            std::vector<TupleT> tuples;
            tuples.reserve(static_cast<size_t>(Size));
            std::copy_if(std::begin(data), std::end(data), std::back_inserter(tuples), [&](const TupleT &v) {
                return (std::get<ColumnId1>(v) == columnValue1) && (std::get<ColumnId2>(v) == columnValue2) && (std::get<ColumnId3>(v) == columnValue3);
            });
            return tuples;
        }

        template <std::size_t ColumnId1, std::size_t ColumnId2>
        [[nodiscard]] auto tuples_matching_at_least_one_columns_values(
            const typename std::tuple_element<ColumnId1, TupleT>::type &columnValue1,
            const typename std::tuple_element<ColumnId2, TupleT>::type &columnValue2) const {

            std::vector<TupleT> tuples;
            tuples.reserve(static_cast<size_t>(Size));
            std::copy_if(std::begin(data), std::end(data), std::back_inserter(tuples), [&](const TupleT &v) {
                return (std::get<ColumnId1>(v) == columnValue1) || (std::get<ColumnId2>(v) == columnValue2);
            });
            return tuples;
        }

        template <std::size_t ColumnId1, std::size_t ColumnId2, std::size_t ColumnId3>
        [[nodiscard]] auto tuples_matching_at_least_one_columns_values(
            const typename std::tuple_element<ColumnId1, TupleT>::type &columnValue1,
            const typename std::tuple_element<ColumnId2, TupleT>::type &columnValue2,
            const typename std::tuple_element<ColumnId3, TupleT>::type &columnValue3) const {

            std::vector<TupleT> tuples;
            tuples.reserve(static_cast<size_t>(Size));
            std::copy_if(std::begin(data), std::end(data), std::back_inserter(tuples), [&](const TupleT &v) {
                return (std::get<ColumnId1>(v) == columnValue1) || (std::get<ColumnId2>(v) == columnValue2) || (std::get<ColumnId3>(v) == columnValue3);
            });
            return tuples;
        }


        // retrieve all lines tuples which column n°ColumnId equals their column value
        template <std::size_t ColumnId>
        [[nodiscard]] auto tuples_not_matching_columns_values(const typename std::tuple_element<ColumnId, TupleT>::type &columnValue) const {
            std::vector<TupleT> tuples;
            tuples.reserve(static_cast<size_t>(Size));
            std::copy_if(std::begin(data), std::end(data), std::back_inserter(tuples), [&](const TupleT &v) {
                return std::get<ColumnId>(v) != columnValue;
            });
            return tuples;
        }

        template <std::size_t ColumnId1, std::size_t ColumnId2>
        [[nodiscard]] auto tuples_not_matching_columns_values(
            const typename std::tuple_element<ColumnId1, TupleT>::type &columnValue1,
            const typename std::tuple_element<ColumnId2, TupleT>::type &columnValue2) const {

            std::vector<TupleT> tuples;
            tuples.reserve(static_cast<size_t>(Size));
            std::copy_if(std::begin(data), std::end(data), std::back_inserter(tuples), [&](const TupleT &v) {
                return (std::get<ColumnId1>(v) != columnValue1) && (std::get<ColumnId2>(v) != columnValue2);
            });
            return tuples;
        }

        template <std::size_t ColumnId1, std::size_t ColumnId2, std::size_t ColumnId3>
        [[nodiscard]] auto tuples_not_matching_columns_values(
            const typename std::tuple_element<ColumnId1, TupleT>::type &columnValue1,
            const typename std::tuple_element<ColumnId2, TupleT>::type &columnValue2,
            const typename std::tuple_element<ColumnId3, TupleT>::type &columnValue3) const {

            std::vector<TupleT> tuples;
            tuples.reserve(static_cast<size_t>(Size));
            std::copy_if(std::begin(data), std::end(data), std::back_inserter(tuples), [&](const TupleT &v) {
                return (std::get<ColumnId1>(v) != columnValue1) && (std::get<ColumnId2>(v) != columnValue2) && (std::get<ColumnId3>(v) != columnValue3);
            });
            return tuples;
        }

        template <std::size_t ColumnIdTypeToCompare, std::size_t ColumnIdTypeToGet>
        [[nodiscard]] auto elements_matching_columns_values(const typename std::tuple_element<ColumnIdTypeToCompare, TupleT>::type &columnValue) const {

            std::vector<typename std::tuple_element<ColumnIdTypeToGet, TupleT>::type> values(count_equal<ColumnIdTypeToCompare>(columnValue));
            size_t id = 0;
            for(const auto &d : data){
                if(std::get<ColumnIdTypeToCompare>(d) == columnValue){
                    values[id++] = std::get<ColumnIdTypeToGet>(d);
                }
            }
            return values;
        }

        template <std::size_t ColumnIdTypeToCompare, std::size_t ColumnIdTypeToGet>
        [[nodiscard]] auto elements_not_matching_columns_values(const typename std::tuple_element<ColumnIdTypeToCompare, TupleT>::type &columnValue) const {

            std::vector<typename std::tuple_element<ColumnIdTypeToGet, TupleT>::type> values(count_not_equal<ColumnIdTypeToCompare>(columnValue));
            size_t id = 0;
            for(const auto &d : data){
                if(std::get<ColumnIdTypeToCompare>(d) != columnValue){
                    values[id++] = std::get<ColumnIdTypeToGet>(d);
                }
            }
            return values;
        }
    };
}




