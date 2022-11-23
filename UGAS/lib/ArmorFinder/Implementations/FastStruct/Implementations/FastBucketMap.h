#pragma once

#include <map>

template <typename Key, typename Value>
class FastBucketMap {
private:
	struct Node {
		Value data;
		Node* next;
	};
	std::map<Key, Node*> _map;
	FastStack<Node> _dataArray;

public:
	inline void Reset() {
		_map.clear();
		_dataArray.Reset();
	}

	inline void Reset(int maxLength) {
		_map.clear();
		_dataArray.Reset(maxLength);
	}

	inline void Add(Key key, Value value) {
		auto iter = _map.find(key);
		if (iter != _map.end()) {
			Node* p = _dataArray.Push({ value, iter->second });
			iter->second = p;
		} 
		else {
			Node* p = _dataArray.Push({ value, nullptr });
			_map.insert(std::map<Key, Node*>::value_type(key, p));
		}
	}
};

