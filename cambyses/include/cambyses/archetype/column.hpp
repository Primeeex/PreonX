#pragma once

#include "cambyses/component/component_registry.hpp"
#include "foundation/core/assert.hpp"
#include "foundation/memory/allocator.hpp"

#include <cstring>

namespace cambyses {

class Column {
public:
    Column() = default;
    explicit Column(ColumnOps ops) : ops_(ops) {}

    Column(const Column&) = delete;
    Column& operator=(const Column&) = delete;

    Column(Column&& other) noexcept
        : data_(other.data_), size_(other.size_), capacity_(other.capacity_), ops_(other.ops_) {
        other.data_ = nullptr;
        other.size_ = 0;
        other.capacity_ = 0;
    }

    Column& operator=(Column&& other) noexcept {
        if (this != &other) {
            clear();
            deallocate_data();
            data_ = other.data_;
            size_ = other.size_;
            capacity_ = other.capacity_;
            ops_ = other.ops_;
            other.data_ = nullptr;
            other.size_ = 0;
            other.capacity_ = 0;
        }
        return *this;
    }

    ~Column() {
        clear();
        deallocate_data();
    }

    void reserve(foundation::size_t new_capacity,
                 foundation::Allocator& alloc = foundation::default_allocator());

    void emplace_default(foundation::Allocator& alloc = foundation::default_allocator());

    void push_back(const void* element,
                   foundation::Allocator& alloc = foundation::default_allocator());

    void swap_remove(foundation::size_t index);

    void clear();

    void copy_element(foundation::size_t dst_idx, const Column& src, foundation::size_t src_idx);
    void move_element(foundation::size_t dst_idx, Column& src, foundation::size_t src_idx);

    template <typename T>
    [[nodiscard]] T& get_as(foundation::size_t index) {
        PREONX_ASSERT(index < size_);
        return *reinterpret_cast<T*>(data_ + index * ops_.element_size);
    }

    template <typename T>
    [[nodiscard]] const T& get_as(foundation::size_t index) const {
        PREONX_ASSERT(index < size_);
        return *reinterpret_cast<const T*>(data_ + index * ops_.element_size);
    }

    [[nodiscard]] void* get(foundation::size_t index) {
        PREONX_ASSERT(index < size_);
        return data_ + index * ops_.element_size;
    }

    [[nodiscard]] const void* get(foundation::size_t index) const {
        PREONX_ASSERT(index < size_);
        return data_ + index * ops_.element_size;
    }

    [[nodiscard]] foundation::size_t size() const noexcept { return size_; }
    [[nodiscard]] foundation::size_t capacity() const noexcept { return capacity_; }
    [[nodiscard]] bool empty() const noexcept { return size_ == 0; }
    [[nodiscard]] const ColumnOps& ops() const noexcept { return ops_; }
    [[nodiscard]] foundation::u8* raw_data() noexcept { return data_; }
    [[nodiscard]] const foundation::u8* raw_data() const noexcept { return data_; }

private:
    void grow_to(foundation::size_t new_capacity, foundation::Allocator& alloc);
    void deallocate_data();

    foundation::u8* data_ = nullptr;
    foundation::size_t size_ = 0;
    foundation::size_t capacity_ = 0;
    ColumnOps ops_ = {};
};

} // namespace cambyses
