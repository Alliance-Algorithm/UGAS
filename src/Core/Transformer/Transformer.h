#pragma once
/*
Creation Date: 2023/06/24
Latest Update: 2023/07/18
Developer(s): 22-Qzh
(C)Copyright: NJUST.Alliance - All rights reserved
Header Functions:
- 一个强类型的tf
*/

#include <eigen3/Eigen/Dense>

namespace transformer {
    struct Root { };

    namespace internal_calculation {
        // Calculate whether T is a legal node.
        // Recursively visit T::Header until the root node.
        // Return true only if each T::Header exists and T eventually points to Root.
        // SFINAE see https://stackoverflow.com/questions/1005476.
        template <typename T, typename = int>
        struct IsLegalNode {
            static constexpr bool value = std::is_same<T, Root>::value;
        };

        template <typename T>
        struct IsLegalNode <T, decltype((void)typename T::Header{}, 0)> {
            static constexpr bool value = IsLegalNode<typename T::Header>::value;
        };
    }

    // forward declaration
    template <typename From, typename To>
    auto CalculateTransform();

    // A struct that inherits frame represents a coordinate system.
    template <typename NodeType>
    struct Frame {
        struct Position {
            using Node = NodeType;

            explicit Position(const Eigen::Vector3d& coord) {
                coordinates = coord;
            }
            Position(const double& x, const double& y, const double& z) {
                coordinates = { x, y, z };
            }
            Position() = default;

            // Enable the implicit conversation between node types.
            // '...enable_if_t...' equals to C++20: 'requires internal_calculation::IsLegalNode<TargetNodeType>::value'.
            template <typename TargetType, std::enable_if_t<internal_calculation::IsLegalNode<typename TargetType::Node>::value, int> = 0>
            operator TargetType() const {
                return TargetType{ CalculateTransform<typename TargetType::Node, NodeType>() * coordinates };
            }

            Eigen::Vector3d& operator*() {
                return coordinates;
            }

            const Eigen::Vector3d& operator*() const {
                return coordinates;
            }

            Eigen::Vector3d* operator->() {
                return &coordinates;
            }

            const Eigen::Vector3d* operator->() const {
                return &coordinates;
            }

            Eigen::Vector3d coordinates;
        };

        struct DirectionVector {
            using Node = NodeType;

            explicit DirectionVector(const Eigen::Vector3d& vec) {
                vector = vec;
            }
            DirectionVector(const double& x, const double& y, const double& z) {
                vector = { x, y, z };
            }
            DirectionVector() = default;

            template <typename TargetType, std::enable_if_t<internal_calculation::IsLegalNode<typename TargetType::Node>::value, int> = 0>
            operator TargetType() const {
                Eigen::Isometry3d transform = CalculateTransform<typename TargetType::Node, NodeType>();
                return TargetType{ transform.linear() * vector };
            }

            Eigen::Vector3d& operator*() {
                return vector;
            }

            const Eigen::Vector3d& operator*() const {
                return vector;
            }

            Eigen::Vector3d* operator->() {
                return &vector;
            }

            const Eigen::Vector3d* operator->() const {
                return &vector;
            }

            Eigen::Vector3d vector;
        };

        struct Rotation {
            using Node = NodeType;

            Rotation() = default;
            explicit Rotation(const Eigen::Quaterniond& quat) : quaternion(quat) { }
            explicit Rotation(const Eigen::AngleAxisd& quat) : quaternion(quat) { }
            Rotation(const double& w, const double& x, const double& y, const double& z) :
                quaternion(w, x, y, z) {
            }

            template <typename TargetType, std::enable_if_t<internal_calculation::IsLegalNode<typename TargetType::Node>::value, int> = 0>
            operator TargetType() const {
                Eigen::Isometry3d transform = CalculateTransform<typename TargetType::Node, NodeType>();
                return TargetType{ Eigen::Quaterniond{ transform.linear() * quaternion } };
            }

            Eigen::Quaterniond& operator*() {
                return quaternion;
            }

            const Eigen::Quaterniond& operator*() const {
                return quaternion;
            }

            Eigen::Quaterniond* operator->() {
                return &quaternion;
            }

            const Eigen::Quaterniond* operator->() const {
                return &quaternion;
            }

            Eigen::Quaterniond quaternion;
        };
    };

    namespace internal_calculation {
        template <typename Node>
        struct GetDepth {
            static constexpr int value = GetDepth<typename Node::Header>::value + 1;
        };

        template <>
        struct GetDepth<Root> {
            static constexpr int value = 0;
        };

        template <int a_depth, int b_depth, typename NodeA, typename NodeB>
        constexpr auto GetLcaRecursion(const NodeA&, const NodeB&) {
            if constexpr (std::is_same<NodeA, NodeB>::value)
                return NodeA{};
            else {
                if constexpr (a_depth > b_depth)
                    return GetLcaRecursion<a_depth - 1, b_depth>(typename NodeA::Header{}, NodeB{});
                else if constexpr (b_depth > a_depth)
                    return GetLcaRecursion<a_depth, b_depth - 1>(NodeA{}, typename NodeB::Header{});
                else
                    return GetLcaRecursion<a_depth - 1, b_depth - 1>(typename NodeA::Header{}, typename NodeB::Header{});
            }
        }

        template <int a_depth, int b_depth, typename NodeB>
        constexpr auto GetLcaRecursion(const Root&, const NodeB&) {
            return Root{};
        }

        template <int a_depth, int b_depth, typename NodeA>
        constexpr auto GetLcaRecursion(const NodeA&, const Root&) {
            return Root{};
        }

        template <int a_depth, int b_depth>
        constexpr auto GetLcaRecursion(const Root&, const Root&) {
            return Root{};
        }

        template <typename NodeA, typename NodeB>
        struct GetLCA {
        private:
            static constexpr auto Calculate() {
                constexpr int a_depth = GetDepth<NodeA>::value;
                constexpr int b_depth = GetDepth<NodeB>::value;
                return GetLcaRecursion<a_depth, b_depth>(NodeA{}, NodeB{});
            }
        public:
            using Result = decltype(Calculate());
        };

        template <typename Node, typename LCA>
        struct TransformToLCA {
            static auto Calculate() {
                return TransformToLCA<typename Node::Header, LCA>::Calculate() * Node::transform;
            }
        };

        template <typename Node>
        struct TransformToLCA<Node, typename Node::Header> {
            static auto Calculate() {
                return Node::transform;
            }
        };
    }

    template <typename From, typename To, typename T>
    void SetRotation(const T& value) {
        static_assert(!(std::is_same<From, Root>::value || std::is_same<To, Root>::value),
                      "It is illegal to set the transformation to the Root frame.");

        constexpr bool forward = std::is_same<typename To::Header, From>::value;
        constexpr bool backward = std::is_same<typename From::Header, To>::value;
        static_assert(forward && !backward, "The transformation is from child to header, please exchange 'From' & 'To'.");
        static_assert(forward || backward, "There is no direct connection between two frame.");
        static_assert(!(forward && backward), "The two frames are headers for each other.");

        if constexpr (forward) {
            To::transform.linear() = value.toRotationMatrix();
        }
    }

    template <typename From, typename To, typename T>
    void SetTranslation(T&& value) {
        static_assert(!(std::is_same<From, Root>::value || std::is_same<To, Root>::value),
                      "It is illegal to set the transformation to the Root frame.");

        constexpr bool forward = std::is_same<typename To::Header, From>::value;
        constexpr bool backward = std::is_same<typename From::Header, To>::value;
        static_assert(forward && !backward, "The transformation is from child to header, please exchange 'From' & 'To'.");
        static_assert(forward || backward, "There is no direct connection between two frame.");
        static_assert(!(forward && backward), "The two frames are headers for each other.");

        if constexpr (forward) {
            To::transform.translation() = std::forward<T>(value);
        }
    }

    template <typename From, typename To, typename T>
    void SetTransform(T&& value) {
        static_assert(!(std::is_same<From, Root>::value || std::is_same<To, Root>::value),
                      "It is illegal to set the transformation to the Root frame.");

        constexpr bool forward = std::is_same<typename To::Header, From>::value;
        constexpr bool backward = std::is_same<typename From::Header, To>::value;
        static_assert(forward && !backward, "The transformation is from child to header, please exchange 'From' & 'To'.");
        static_assert(forward || backward, "There is no direct connection between two frame.");
        static_assert(!(forward && backward), "The two frames are headers for each other.");

        if constexpr (forward) {
            To::transform = std::forward<T>(value);
        }
    }

    // Get the transformation between directly connected nodes.
    template <typename From, typename To>
    auto& GetTransform() {
        static_assert(!(std::is_same<From, Root>::value || std::is_same<To, Root>::value),
                      "It is illegal to set the transformation to the Root frame.");

        constexpr bool forward = std::is_same<typename To::Header, From>::value;
        constexpr bool backward = std::is_same<typename From::Header, To>::value;
        static_assert(forward && !backward, "The transformation is from child to header, please exchange 'From' & 'To'.");
        static_assert(forward || backward, "There is no direct connection between two frame.");
        static_assert(!(forward && backward), "The two frames are headers for each other.");

        if constexpr (forward) {
            return To::transform;
        }
    }

    // Calculate the transformation between nodes, NO direct connection required.
    template <typename From, typename To>
    auto CalculateTransform() {
        using LCA = typename internal_calculation::GetLCA<From, To>::Result;
        static_assert(!std::is_same<LCA, Root>::value, "Trying to convert between two separated tf trees.");
        if constexpr (!std::is_same<LCA, From>::value && !std::is_same<LCA, To>::value) {
            auto t1 = internal_calculation::TransformToLCA<From, LCA>::Calculate();
            auto t2 = internal_calculation::TransformToLCA<To, LCA>::Calculate();
            return t1.inverse() * t2;
        }
        else if constexpr (std::is_same<LCA, From>::value && std::is_same<LCA, To>::value) {
            return Eigen::Translation3d::Identity();
        }
        else if constexpr (std::is_same<LCA, From>::value) {
            auto t2 = internal_calculation::TransformToLCA<To, LCA>::Calculate();
            return t2;
        }
        else if constexpr (std::is_same<LCA, To>::value) {
            auto t1 = internal_calculation::TransformToLCA<From, LCA>::Calculate();
            return t1.inverse();
        }
    }

    using Transform = Eigen::Isometry3d;
    using Rotation = Eigen::Isometry3d;
    using Translation = Eigen::Translation3d;
}
