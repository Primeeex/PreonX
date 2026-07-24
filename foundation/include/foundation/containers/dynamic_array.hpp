#pragma once

#include "foundation/core/assert.hpp"
#include "foundation/core/types.hpp"
#include "foundation/memory/allocator.hpp"

#include <algorithm>
#include <concepts>
#include <cstring>
#include <initializer_list>
#include <iterator>
#include <memory>
#include <type_traits>
#include <utility>

namespace foundation {

template <typename T>
class DynamicArray {
public:
    using value_type = T;
    using size_type = size_t;
    using difference_type = ptrdiff_t;
    using reference = T&;
    using const_reference = const T&;
    using pointer = T*;
    using const_pointer = const T*;
    using iterator = T*;
    using const_iterator = const T*;
    using reverse_iterator = std::reverse_iterator<iterator>;
    using const_reverse_iterator = std::reverse_iterator<const_iterator>;

    DynamicArray() noexcept = default;

    explicit DynamicArray(size_type count, Allocator& allocator = default_allocator())
        : allocator_(&allocator) {
        if (count > 0) {
            grow_to(count);
            construct_range(data_, count);
        }
        size_ = count;
    }

    DynamicArray(size_type count, const T& value, Allocator& allocator = default_allocator())
        : allocator_(&allocator) {
        if (count > 0) {
            grow_to(count);
            construct_range(data_, count, value);
        }
        size_ = count;
    }

    template <std::input_iterator InputIt>
    DynamicArray(InputIt first, InputIt last, Allocator& allocator = default_allocator())
        : allocator_(&allocator) {
        auto count = static_cast<size_type>(std::distance(first, last));
        if (count > 0) {
            grow_to(count);
            for (size_type i = 0; i < count; ++i) {
                ::new (data_ + i) T(*first);
                ++first;
            }
        }
        size_ = count;
    }

    DynamicArray(std::initializer_list<T> init, Allocator& allocator = default_allocator())
        : allocator_(&allocator) {
        auto count = init.size();
        if (count > 0) {
            grow_to(count);
            size_type i = 0;
            for (const auto& val : init) {
                ::new (data_ + i) T(val);
                ++i;
            }
        }
        size_ = count;
    }

    DynamicArray(const DynamicArray& other)
        : allocator_(other.allocator_) {
        if (other.size_ > 0) {
            grow_to(other.size_);
            copy_construct_range(data_, other.data_, other.size_);
        }
        size_ = other.size_;
    }

    DynamicArray(DynamicArray&& other) noexcept
        : data_(other.data_), size_(other.size_), capacity_(other.capacity_), allocator_(other.allocator_) {
        other.data_ = nullptr;
        other.size_ = 0;
        other.capacity_ = 0;
    }

    DynamicArray& operator=(const DynamicArray& other) {
        if (this != &other) {
            clear();
            allocator_ = other.allocator_;
            if (other.size_ > 0) {
                grow_to(other.size_);
                copy_construct_range(data_, other.data_, other.size_);
            }
            size_ = other.size_;
        }
        return *this;
    }

    DynamicArray& operator=(DynamicArray&& other) noexcept {
        if (this != &other) {
            destroy_all();
            deallocate_memory();

            data_ = other.data_;
            size_ = other.size_;
            capacity_ = other.capacity_;
            allocator_ = other.allocator_;

            other.data_ = nullptr;
            other.size_ = 0;
            other.capacity_ = 0;
        }
        return *this;
    }

    DynamicArray& operator=(std::initializer_list<T> init) {
        clear();
        auto count = init.size();
        if (count > 0) {
            grow_to(count);
            size_type i = 0;
            for (const auto& val : init) {
                ::new (data_ + i) T(val);
                ++i;
            }
        }
        size_ = count;
        return *this;
    }

    ~DynamicArray() {
        destroy_all();
        deallocate_memory();
    }

    // ── Element access ────────────────────────────────────────────────────────

    [[nodiscard]] reference operator[](size_type index) {
        PREONX_ASSERT_MSG(index < size_, "Index out of bounds");
        return data_[index];
    }

    [[nodiscard]] const_reference operator[](size_type index) const {
        PREONX_ASSERT_MSG(index < size_, "Index out of bounds");
        return data_[index];
    }

    [[nodiscard]] reference at(size_type index) {
        if (index >= size_) {
            PREONX_ASSERT_MSG(false, "Index out of bounds");
        }
        return data_[index];
    }

    [[nodiscard]] const_reference at(size_type index) const {
        if (index >= size_) {
            PREONX_ASSERT_MSG(false, "Index out of bounds");
        }
        return data_[index];
    }

    [[nodiscard]] reference front() {
        PREONX_ASSERT_MSG(size_ > 0, "front() called on empty array");
        return data_[0];
    }

    [[nodiscard]] const_reference front() const {
        PREONX_ASSERT_MSG(size_ > 0, "front() called on empty array");
        return data_[0];
    }

    [[nodiscard]] reference back() {
        PREONX_ASSERT_MSG(size_ > 0, "back() called on empty array");
        return data_[size_ - 1];
    }

    [[nodiscard]] const_reference back() const {
        PREONX_ASSERT_MSG(size_ > 0, "back() called on empty array");
        return data_[size_ - 1];
    }

    [[nodiscard]] pointer data() noexcept {
        return data_;
    }

    [[nodiscard]] const_pointer data() const noexcept {
        return data_;
    }

    // ── Size and capacity ─────────────────────────────────────────────────────

    [[nodiscard]] bool empty() const noexcept {
        return size_ == 0;
    }

    [[nodiscard]] size_type size() const noexcept {
        return size_;
    }

    [[nodiscard]] size_type capacity() const noexcept {
        return capacity_;
    }

    void reserve(size_type new_capacity) {
        if (new_capacity > capacity_) {
            grow_to(new_capacity);
        }
    }

    void shrink_to_fit() {
        if (capacity_ > size_) {
            reallocate(size_);
        }
    }

    // ── Iterators ─────────────────────────────────────────────────────────────

    [[nodiscard]] iterator begin() noexcept {
        return data_;
    }

    [[nodiscard]] const_iterator begin() const noexcept {
        return data_;
    }

    [[nodiscard]] const_iterator cbegin() const noexcept {
        return data_;
    }

    [[nodiscard]] iterator end() noexcept {
        return data_ + size_;
    }

    [[nodiscard]] const_iterator end() const noexcept {
        return data_ + size_;
    }

    [[nodiscard]] const_iterator cend() const noexcept {
        return data_ + size_;
    }

    [[nodiscard]] reverse_iterator rbegin() noexcept {
        return reverse_iterator(end());
    }

    [[nodiscard]] const_reverse_iterator rbegin() const noexcept {
        return const_reverse_iterator(end());
    }

    [[nodiscard]] reverse_iterator rend() noexcept {
        return reverse_iterator(begin());
    }

    [[nodiscard]] const_reverse_iterator rend() const noexcept {
        return const_reverse_iterator(begin());
    }

    // ── Modifiers ─────────────────────────────────────────────────────────────

    void clear() noexcept {
        destroy_all();
        size_ = 0;
    }

    iterator insert(const_iterator pos, const T& value) {
        auto offset = static_cast<size_type>(pos - data_);
        const size_type count = 1;
        if (size_ + count > capacity_) {
            grow_to(calc_new_capacity(size_ + count));
        }
        shift_right(static_cast<size_type>(offset), count);
        ::new (data_ + offset) T(value);
        ++size_;
        return data_ + offset;
    }

    iterator insert(const_iterator pos, T&& value) {
        auto offset = static_cast<size_type>(pos - data_);
        const size_type count = 1;
        if (size_ + count > capacity_) {
            grow_to(calc_new_capacity(size_ + count));
        }
        shift_right(static_cast<size_type>(offset), count);
        ::new (data_ + offset) T(std::move(value));
        ++size_;
        return data_ + offset;
    }

    iterator insert(const_iterator pos, size_type count, const T& value) {
        auto offset = static_cast<size_type>(pos - data_);
        if (size_ + count > capacity_) {
            grow_to(calc_new_capacity(size_ + count));
        }
        shift_right(static_cast<size_type>(offset), count);
        for (size_type i = 0; i < count; ++i) {
            ::new (data_ + offset + i) T(value);
        }
        size_ += count;
        return data_ + offset;
    }

    template <std::input_iterator InputIt>
    iterator insert(const_iterator pos, InputIt first, InputIt last) {
        auto offset = static_cast<size_type>(pos - data_);
        auto count = static_cast<size_type>(std::distance(first, last));
        if (size_ + count > capacity_) {
            grow_to(calc_new_capacity(size_ + count));
        }
        shift_right(static_cast<size_type>(offset), count);
        for (size_type i = 0; i < count; ++i) {
            ::new (data_ + offset + i) T(*first);
            ++first;
        }
        size_ += count;
        return data_ + offset;
    }

    iterator erase(const_iterator pos) {
        auto offset = static_cast<size_type>(pos - data_);
        PREONX_ASSERT_MSG(offset < size_, "Iterator out of bounds");
        (data_ + offset)->~T();
        shift_left(offset, 1);
        --size_;
        return data_ + offset;
    }

    iterator erase(const_iterator first, const_iterator last) {
        auto offset = static_cast<size_type>(first - data_);
        auto count = static_cast<size_type>(last - first);
        PREONX_ASSERT_MSG(offset + count <= size_, "Iterator range out of bounds");
        for (size_type i = 0; i < count; ++i) {
            (data_ + offset + i)->~T();
        }
        shift_left(offset, count);
        size_ -= count;
        return data_ + offset;
    }

    void push_back(const T& value) {
        if (size_ == capacity_) {
            grow_to(calc_new_capacity(size_ + 1));
        }
        ::new (data_ + size_) T(value);
        ++size_;
    }

    void push_back(T&& value) {
        if (size_ == capacity_) {
            grow_to(calc_new_capacity(size_ + 1));
        }
        ::new (data_ + size_) T(std::move(value));
        ++size_;
    }

    template <typename... Args>
        requires std::constructible_from<T, Args...>
    reference emplace_back(Args&&... args) {
        if (size_ == capacity_) {
            grow_to(calc_new_capacity(size_ + 1));
        }
        ::new (data_ + size_) T(std::forward<Args>(args)...);
        ++size_;
        return data_[size_ - 1];
    }

    void pop_back() {
        PREONX_ASSERT_MSG(size_ > 0, "pop_back() called on empty array");
        --size_;
        (data_ + size_)->~T();
    }

    void resize(size_type new_size) {
        if (new_size > size_) {
            if (new_size > capacity_) {
                grow_to(new_size);
            }
            construct_range(data_ + size_, new_size - size_);
        } else if (new_size < size_) {
            for (size_type i = new_size; i < size_; ++i) {
                (data_ + i)->~T();
            }
        }
        size_ = new_size;
    }

    void resize(size_type new_size, const T& value) {
        if (new_size > size_) {
            if (new_size > capacity_) {
                grow_to(new_size);
            }
            construct_range(data_ + size_, new_size - size_, value);
        } else if (new_size < size_) {
            for (size_type i = new_size; i < size_; ++i) {
                (data_ + i)->~T();
            }
        }
        size_ = new_size;
    }

    void swap(DynamicArray& other) noexcept {
        if (this != &other) {
            std::swap(data_, other.data_);
            std::swap(size_, other.size_);
            std::swap(capacity_, other.capacity_);
            std::swap(allocator_, other.allocator_);
        }
    }

    // ── Search ────────────────────────────────────────────────────────────────

    [[nodiscard]] size_type find(const T& value) const {
        for (size_type i = 0; i < size_; ++i) {
            if (data_[i] == value) {
                return i;
            }
        }
        return kNotFound;
    }

    [[nodiscard]] bool contains(const T& value) const {
        return find(value) != kNotFound;
    }

private:
    void grow_to(size_type new_capacity) {
        reallocate(new_capacity);
    }

    void reallocate(size_type new_capacity) {
        pointer new_data = static_cast<pointer>(
            allocator_->allocate(new_capacity * sizeof(T), alignof(T)));

        if constexpr (std::is_trivially_copyable_v<T>) {
            if (data_) {
                std::memcpy(new_data, data_, size_ * sizeof(T));
            }
        } else {
            for (size_type i = 0; i < size_; ++i) {
                ::new (new_data + i) T(std::move(data_[i]));
                (data_ + i)->~T();
            }
        }

        if (data_) {
            allocator_->deallocate(data_, capacity_ * sizeof(T), alignof(T));
        }

        data_ = new_data;
        capacity_ = new_capacity;
    }

    [[nodiscard]] size_type calc_new_capacity(size_type required) const {
        size_type new_capacity = capacity_ == 0 ? 8 : capacity_;
        while (new_capacity < required) {
            new_capacity += (new_capacity >> 1) + 1;
        }
        return new_capacity;
    }

    void destroy_all() {
        if constexpr (!std::is_trivially_destructible_v<T>) {
            for (size_type i = 0; i < size_; ++i) {
                (data_ + i)->~T();
            }
        }
    }

    void deallocate_memory() {
        if (data_) {
            allocator_->deallocate(data_, capacity_ * sizeof(T), alignof(T));
            data_ = nullptr;
            capacity_ = 0;
        }
    }

    void construct_range(pointer dest, size_type count) {
        if constexpr (std::is_trivially_constructible_v<T>) {
            std::memset(dest, 0, count * sizeof(T));
        } else {
            for (size_type i = 0; i < count; ++i) {
                ::new (dest + i) T();
            }
        }
    }

    void construct_range(pointer dest, size_type count, const T& value) {
        for (size_type i = 0; i < count; ++i) {
            ::new (dest + i) T(value);
        }
    }

    void copy_construct_range(pointer dest, const_pointer src, size_type count) {
        if constexpr (std::is_trivially_copyable_v<T>) {
            std::memcpy(dest, src, count * sizeof(T));
        } else {
            for (size_type i = 0; i < count; ++i) {
                ::new (dest + i) T(src[i]);
            }
        }
    }

    void shift_right(size_type offset, size_type count) {
        if constexpr (std::is_trivially_copyable_v<T>) {
            std::memmove(data_ + offset + count, data_ + offset, (size_ - offset) * sizeof(T));
        } else {
            for (size_type i = size_; i > offset; --i) {
                ::new (data_ + i + count - 1) T(std::move(data_[i - 1]));
                (data_ + i - 1)->~T();
            }
        }
    }

    void shift_left(size_type offset, size_type count) {
        if constexpr (std::is_trivially_copyable_v<T>) {
            std::memmove(data_ + offset, data_ + offset + count, (size_ - offset - count) * sizeof(T));
        } else {
            for (size_type i = offset; i < size_ - count; ++i) {
                (data_ + i)->~T();
                ::new (data_ + i) T(std::move(data_[i + count]));
                (data_ + i + count)->~T();
            }
        }
    }

    pointer data_ = nullptr;
    size_type size_ = 0;
    size_type capacity_ = 0;
    Allocator* allocator_ = &default_allocator();
};

template <typename T>
[[nodiscard]] bool operator==(const DynamicArray<T>& a, const DynamicArray<T>& b) {
    if (a.size() != b.size()) return false;
    for (typename DynamicArray<T>::size_type i = 0; i < a.size(); ++i) {
        if (a[i] != b[i]) return false;
    }
    return true;
}

template <typename T>
[[nodiscard]] bool operator!=(const DynamicArray<T>& a, const DynamicArray<T>& b) {
    return !(a == b);
}

} // namespace foundation
