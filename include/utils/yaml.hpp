#pragma once

#include <Eigen/Dense>

#include <yaml-cpp/yaml.h>

// Enables node.as<Eigen::Vector3d>() and friends.
//   Vectors (any Eigen::Matrix with one column) are flat sequences:  key: [1.0, 2.0, 3.0]
//   Other matrices are sequences of row sequences:                   key: [[1, 0], [0, 1]]
// Fixed-size types must match the sequence length exactly; dynamic dimensions take whatever is given.
namespace YAML {
template <typename Scalar, int Rows, int Cols, int Options, int MaxRows, int MaxCols>
struct convert<Eigen::Matrix<Scalar, Rows, Cols, Options, MaxRows, MaxCols>> {
    using Type = Eigen::Matrix<Scalar, Rows, Cols, Options, MaxRows, MaxCols>;

    static Node encode(const Type& value) {
        Node node(NodeType::Sequence);
        if constexpr (Cols == 1) {
            for (Eigen::Index i = 0; i < value.size(); ++i) {
                node.push_back(value(i));
            }
        } else {
            for (Eigen::Index r = 0; r < value.rows(); ++r) {
                Node row(NodeType::Sequence);
                for (Eigen::Index c = 0; c < value.cols(); ++c) {
                    row.push_back(value(r, c));
                }
                node.push_back(row);
            }
        }
        return node;
    }

    static bool decode(const Node& node, Type& value) {
        if (!node.IsSequence()) {
            return false;
        }

        if constexpr (Cols == 1) {
            const Eigen::Index size = static_cast<Eigen::Index>(node.size());
            if (Rows != Eigen::Dynamic && size != Rows) {
                return false;
            }
            value.resize(size, 1);
            for (Eigen::Index i = 0; i < size; ++i) {
                value(i) = node[i].template as<Scalar>();
            }
            return true;
        } else {
            const Eigen::Index rows = static_cast<Eigen::Index>(node.size());
            if (rows == 0 || (Rows != Eigen::Dynamic && rows != Rows)) {
                return false;
            }
            for (Eigen::Index r = 0; r < rows; ++r) {
                if (!node[r].IsSequence()) {
                    return false;
                }
            }
            const Eigen::Index cols = static_cast<Eigen::Index>(node[0].size());
            if (Cols != Eigen::Dynamic && cols != Cols) {
                return false;
            }
            value.resize(rows, cols);
            for (Eigen::Index r = 0; r < rows; ++r) {
                if (static_cast<Eigen::Index>(node[r].size()) != cols) {
                    return false;
                }
                for (Eigen::Index c = 0; c < cols; ++c) {
                    value(r, c) = node[r][c].template as<Scalar>();
                }
            }
            return true;
        }
    }
};
}  // namespace YAML
