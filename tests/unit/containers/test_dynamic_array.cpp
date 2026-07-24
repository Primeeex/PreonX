#include <gtest/gtest.h>

#include "foundation/containers/dynamic_array.hpp"

#include <string>

namespace {

TEST(DynamicArrayTest, DefaultConstruction) {
    foundation::DynamicArray<int> arr;
    EXPECT_TRUE(arr.empty());
    EXPECT_EQ(arr.size(), 0u);
    EXPECT_EQ(arr.capacity(), 0u);
}

TEST(DynamicArrayTest, SizeConstruction) {
    foundation::DynamicArray<int> arr(10);
    EXPECT_EQ(arr.size(), 10u);
    EXPECT_GE(arr.capacity(), 10u);
    for (size_t i = 0; i < 10; ++i) {
        EXPECT_EQ(arr[i], 0);
    }
}

TEST(DynamicArrayTest, ValueConstruction) {
    foundation::DynamicArray<int> arr(5, 42);
    EXPECT_EQ(arr.size(), 5u);
    for (size_t i = 0; i < 5; ++i) {
        EXPECT_EQ(arr[i], 42);
    }
}

TEST(DynamicArrayTest, InitializerList) {
    foundation::DynamicArray<int> arr = {1, 2, 3, 4, 5};
    EXPECT_EQ(arr.size(), 5u);
    EXPECT_EQ(arr[0], 1);
    EXPECT_EQ(arr[4], 5);
}

TEST(DynamicArrayTest, CopyConstruction) {
    foundation::DynamicArray<int> original = {1, 2, 3};
    foundation::DynamicArray<int> copy(original);
    EXPECT_EQ(copy.size(), 3u);
    EXPECT_EQ(copy[0], 1);
    EXPECT_EQ(copy[1], 2);
    EXPECT_EQ(copy[2], 3);
}

TEST(DynamicArrayTest, MoveConstruction) {
    foundation::DynamicArray<int> original = {1, 2, 3};
    foundation::DynamicArray<int> moved(std::move(original));
    EXPECT_EQ(moved.size(), 3u);
    EXPECT_EQ(moved[0], 1);
    EXPECT_TRUE(original.empty());
}

TEST(DynamicArrayTest, PushBack) {
    foundation::DynamicArray<int> arr;
    arr.push_back(1);
    arr.push_back(2);
    arr.push_back(3);
    EXPECT_EQ(arr.size(), 3u);
    EXPECT_EQ(arr[0], 1);
    EXPECT_EQ(arr[2], 3);
}

TEST(DynamicArrayTest, PushBackGrowth) {
    foundation::DynamicArray<int> arr;
    for (int i = 0; i < 1000; ++i) {
        arr.push_back(i);
    }
    EXPECT_EQ(arr.size(), 1000u);
    for (int i = 0; i < 1000; ++i) {
        EXPECT_EQ(arr[static_cast<size_t>(i)], i);
    }
}

TEST(DynamicArrayTest, EmplaceBack) {
    foundation::DynamicArray<std::string> arr;
    arr.emplace_back("hello");
    arr.emplace_back(5u, 'x');
    EXPECT_EQ(arr.size(), 2u);
    EXPECT_EQ(arr[0], "hello");
    EXPECT_EQ(arr[1], "xxxxx");
}

TEST(DynamicArrayTest, PopBack) {
    foundation::DynamicArray<int> arr = {1, 2, 3};
    arr.pop_back();
    EXPECT_EQ(arr.size(), 2u);
    EXPECT_EQ(arr[0], 1);
    EXPECT_EQ(arr[1], 2);
}

TEST(DynamicArrayTest, Clear) {
    foundation::DynamicArray<int> arr = {1, 2, 3, 4, 5};
    arr.clear();
    EXPECT_TRUE(arr.empty());
    EXPECT_EQ(arr.size(), 0u);
}

TEST(DynamicArrayTest, Iterators) {
    foundation::DynamicArray<int> arr = {10, 20, 30};
    int sum = 0;
    for (int val : arr) {
        sum += val;
    }
    EXPECT_EQ(sum, 60);
}

TEST(DynamicArrayTest, ConstIterators) {
    const foundation::DynamicArray<int> arr = {10, 20, 30};
    int sum = 0;
    for (int val : arr) {
        sum += val;
    }
    EXPECT_EQ(sum, 60);
}

TEST(DynamicArrayTest, FrontBack) {
    foundation::DynamicArray<int> arr = {1, 2, 3};
    EXPECT_EQ(arr.front(), 1);
    EXPECT_EQ(arr.back(), 3);
}

TEST(DynamicArrayTest, Data) {
    foundation::DynamicArray<int> arr = {1, 2, 3};
    const int* data = arr.data();
    EXPECT_EQ(data[0], 1);
    EXPECT_EQ(data[1], 2);
    EXPECT_EQ(data[2], 3);
}

TEST(DynamicArrayTest, Reserve) {
    foundation::DynamicArray<int> arr;
    arr.reserve(100);
    EXPECT_GE(arr.capacity(), 100u);
    EXPECT_TRUE(arr.empty());
}

TEST(DynamicArrayTest, ShrinkToFit) {
    foundation::DynamicArray<int> arr;
    arr.reserve(100);
    arr.push_back(1);
    arr.push_back(2);
    arr.shrink_to_fit();
    EXPECT_EQ(arr.capacity(), 2u);
}

TEST(DynamicArrayTest, Insert) {
    foundation::DynamicArray<int> arr = {1, 3, 4};
    arr.insert(arr.begin() + 1, 2);
    EXPECT_EQ(arr.size(), 4u);
    EXPECT_EQ(arr[0], 1);
    EXPECT_EQ(arr[1], 2);
    EXPECT_EQ(arr[2], 3);
    EXPECT_EQ(arr[3], 4);
}

TEST(DynamicArrayTest, Erase) {
    foundation::DynamicArray<int> arr = {1, 2, 3, 4, 5};
    arr.erase(arr.begin() + 2); // remove 3
    EXPECT_EQ(arr.size(), 4u);
    EXPECT_EQ(arr[0], 1);
    EXPECT_EQ(arr[1], 2);
    EXPECT_EQ(arr[2], 4);
    EXPECT_EQ(arr[3], 5);
}

TEST(DynamicArrayTest, EraseRange) {
    foundation::DynamicArray<int> arr = {1, 2, 3, 4, 5};
    arr.erase(arr.begin() + 1, arr.begin() + 3); // remove 2, 3
    EXPECT_EQ(arr.size(), 3u);
    EXPECT_EQ(arr[0], 1);
    EXPECT_EQ(arr[1], 4);
    EXPECT_EQ(arr[2], 5);
}

TEST(DynamicArrayTest, Find) {
    foundation::DynamicArray<int> arr = {10, 20, 30};
    EXPECT_EQ(arr.find(20), 1u);
    EXPECT_EQ(arr.find(40), foundation::kNotFound);
}

TEST(DynamicArrayTest, Contains) {
    foundation::DynamicArray<int> arr = {10, 20, 30};
    EXPECT_TRUE(arr.contains(20));
    EXPECT_FALSE(arr.contains(40));
}

TEST(DynamicArrayTest, Resize) {
    foundation::DynamicArray<int> arr = {1, 2, 3};
    arr.resize(5);
    EXPECT_EQ(arr.size(), 5u);
    EXPECT_EQ(arr[0], 1);
    EXPECT_EQ(arr[3], 0);
    EXPECT_EQ(arr[4], 0);

    arr.resize(2);
    EXPECT_EQ(arr.size(), 2u);
}

TEST(DynamicArrayTest, Equality) {
    foundation::DynamicArray<int> a = {1, 2, 3};
    foundation::DynamicArray<int> b = {1, 2, 3};
    foundation::DynamicArray<int> c = {1, 2, 4};

    EXPECT_TRUE(a == b);
    EXPECT_TRUE(a != c);
    EXPECT_FALSE(a == c);
}

TEST(DynamicArrayTest, Swap) {
    foundation::DynamicArray<int> a = {1, 2, 3};
    foundation::DynamicArray<int> b = {4, 5};
    a.swap(b);
    EXPECT_EQ(a.size(), 2u);
    EXPECT_EQ(b.size(), 3u);
    EXPECT_EQ(a[0], 4);
    EXPECT_EQ(b[0], 1);
}

TEST(DynamicArrayTest, Assignment) {
    foundation::DynamicArray<int> a = {1, 2, 3};
    foundation::DynamicArray<int> b;
    b = a;
    EXPECT_EQ(b.size(), 3u);
    EXPECT_EQ(b[0], 1);
}

TEST(DynamicArrayTest, MoveAssignment) {
    foundation::DynamicArray<int> a = {1, 2, 3};
    foundation::DynamicArray<int> b;
    b = std::move(a);
    EXPECT_EQ(b.size(), 3u);
    EXPECT_TRUE(a.empty());
}

TEST(DynamicArrayTest, InitializerListAssignment) {
    foundation::DynamicArray<int> arr;
    arr = {10, 20, 30};
    EXPECT_EQ(arr.size(), 3u);
    EXPECT_EQ(arr[0], 10);
    EXPECT_EQ(arr[1], 20);
    EXPECT_EQ(arr[2], 30);
}

TEST(DynamicArrayTest, ManyElementsStress) {
    foundation::DynamicArray<int> arr;
    for (int i = 0; i < 10000; ++i) {
        arr.push_back(i);
    }
    EXPECT_EQ(arr.size(), 10000u);
    for (int i = 0; i < 10000; ++i) {
        EXPECT_EQ(arr[static_cast<size_t>(i)], i);
    }
    arr.clear();
    EXPECT_TRUE(arr.empty());
}

} // namespace
