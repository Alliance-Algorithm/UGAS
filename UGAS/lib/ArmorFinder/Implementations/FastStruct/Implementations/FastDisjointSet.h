#pragma once
#include <map>
#include <vector>

template <class T>
class FastDisjointSet {
public:
	struct Node {
		Node* Parent;
		T Data;
		unsigned char Level;
	};
	struct Group {
		std::vector<T> Items;
		unsigned char Level;
	};
private:
	FastStack<Node> _dataArray;
public:
	const int& Length = _dataArray.Length;

	inline void Reset() {
		_dataArray.Reset();
	}

	inline void Reset(int maxLength) {
		_dataArray.Reset(maxLength);
	}

	inline Node* Add(T data) {
		return _dataArray.Push(Node{ nullptr, data, 0 });
	}

	Node* FindRoot(Node* node) {
		if (node->Parent != nullptr) {
			Node* parent = FindRoot(node->Parent);
			node->Parent = parent;
			return parent;
		}
		else return node;
	}

	inline void SetLevel(Node* node, unsigned char level) {
		FindRoot(node)->Level = level;
	}

	inline void Union(Node* nodeA, Node* nodeB) {
		nodeA = FindRoot(nodeA);
		nodeB = FindRoot(nodeB);
		if (nodeA != nodeB) {
			nodeB->Parent = nodeA;
			if (nodeB->Level > nodeA->Level)
				nodeA->Level = nodeB->Level;
		}
	}

	inline std::vector<std::vector<T>> GetGroups(int scanBefore = -1) {
		if (scanBefore == -1)
			scanBefore = Length;
		auto groups = std::map<Node*, Group>();
		for (int i = 0; i < scanBefore; ++i) {
			Node* node = FindRoot(&_dataArray[i]);
			if (node->Level > 0) continue;
			auto iter = groups.find(node);
			if (iter != groups.end())
				iter->second.Items.push_back(_dataArray[i].Data);
			else
				groups.insert(std::map<Node*, Group>::value_type(node, { {_dataArray[i].Data}, node->Level }));
		}

		std::vector<std::vector<T>> result;
		for (auto iter = groups.begin(); iter != groups.end(); ++iter) {
			auto& array = iter->second.Items;
			result.push_back(array);
		}
		return result;
	}

	inline T& operator[](int index) {
		return _dataArray[index].Data;
	}

	inline const T& operator[](int index) const {
		return _dataArray[index].Data;
	}
};
