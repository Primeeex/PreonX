#include "cambyses/archetype/column.hpp"

#include <algorithm>
#include <cstring>

namespace cambyses {

void Column::reserve(foundation::size_t new_capacity, foundation::Allocator& alloc) {
    if (new_capacity <= capacity_) {
        return;
    }
    grow_to(new_capacity, alloc);
}

void Column::emplace_default(foundation::Allocator& alloc) {
    if (size_ == capacity_) {
        grow_to(capacity_ == 0 ? 8 : capacity_ * 2, alloc);
    }
    foundation::u8* ptr = data_ + size_ * ops_.element_size;
    if (ops_.trivial_construct) {
        std::memset(ptr, 0, ops_.element_size);
    } else if (ops_.default_construct) {
        ops_.default_construct(ptr);
    }
    ++size_;
}

void Column::push_back(const void* element, foundation::Allocator& alloc) {
    if (size_ == capacity_) {
        grow_to(capacity_ == 0 ? 8 : capacity_ * 2, alloc);
    }
    foundation::u8* ptr = data_ + size_ * ops_.element_size;
    if (ops_.trivial_copy) {
        std::memcpy(ptr, element, ops_.element_size);
    } else if (ops_.copy_construct) {
        ops_.copy_construct(ptr, element);
    }
    ++size_;
}

void Column::swap_remove(foundation::size_t index) {
    PREONX_ASSERT(index < size_);
    foundation::size_t last = size_ - 1;

    if (index != last) {
        foundation::u8* dst = data_ + index * ops_.element_size;
        foundation::u8* src = data_ + last * ops_.element_size;
        if (ops_.trivial_copy) {
            std::memcpy(dst, src, ops_.element_size);
        } else if (ops_.move_assign) {
            ops_.move_assign(dst, src);
        }
    }

    if (!ops_.trivial_destruct) {
        ops_.destruct(data_ + last * ops_.element_size);
    }
    --size_;
}

void Column::clear() {
    if (!ops_.trivial_destruct) {
        for (foundation::size_t i = 0; i < size_; ++i) {
            ops_.destruct(data_ + i * ops_.element_size);
        }
    }
    size_ = 0;
}

void Column::copy_element(foundation::size_t dst_idx, const Column& src, foundation::size_t src_idx) {
    foundation::u8* dst = data_ + dst_idx * ops_.element_size;
    const foundation::u8* src_ptr = src.data_ + src_idx * ops_.element_size;

    if (!ops_.trivial_destruct) {
        ops_.destruct(dst);
    }

    if (ops_.trivial_copy) {
        std::memcpy(dst, src_ptr, ops_.element_size);
    } else if (ops_.copy_construct) {
        ops_.copy_construct(dst, src_ptr);
    }
}

void Column::move_element(foundation::size_t dst_idx, Column& src, foundation::size_t src_idx) {
    foundation::u8* dst = data_ + dst_idx * ops_.element_size;
    foundation::u8* src_ptr = src.data_ + src_idx * ops_.element_size;

    if (!ops_.trivial_destruct) {
        ops_.destruct(dst);
    }

    if (ops_.trivial_copy) {
        std::memcpy(dst, src_ptr, ops_.element_size);
    } else if (ops_.move_construct) {
        ops_.move_construct(dst, src_ptr);
        if (!src.ops_.trivial_destruct) {
            src.ops_.destruct(src_ptr);
        }
    }
}

void Column::grow_to(foundation::size_t new_capacity, foundation::Allocator& alloc) {
    foundation::size_t new_bytes = new_capacity * ops_.element_size;
    foundation::u8* new_data = static_cast<foundation::u8*>(
        alloc.allocate(new_bytes, ops_.element_alignment));

    if (data_) {
        if (ops_.trivial_copy) {
            std::memcpy(new_data, data_, size_ * ops_.element_size);
        } else {
            for (foundation::size_t i = 0; i < size_; ++i) {
                foundation::size_t offset = i * ops_.element_size;
                ops_.move_construct(new_data + offset, data_ + offset);
                ops_.destruct(data_ + offset);
            }
        }
        deallocate_data();
    }

    data_ = new_data;
    capacity_ = new_capacity;
}

void Column::deallocate_data() {
    if (data_) {
        foundation::default_allocator().deallocate(data_, capacity_ * ops_.element_size,
                                                   ops_.element_alignment);
        data_ = nullptr;
        capacity_ = 0;
    }
}

} // namespace cambyses
