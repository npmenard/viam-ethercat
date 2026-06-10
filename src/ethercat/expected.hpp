#pragma once

// ethercat::Expected -- minimal C++20 stand-in for std::expected (C++23).
//
// #38 ruling (user, via team-lead): the locked Cia402Fsm API spells
// std::expected<void, FsmError>, but std::expected requires C++23 and the
// project stays pinned at C++20 (the C++23 bump's verified infra cost -- image
// toolchain gcc-11->12 + a boost-1.74 header workaround -- was ruled not worth
// it). This carries the same SHAPE with MIRRORED member names, so the future
// flip back is mechanical:
//     s/ethercat::Expected/std::expected/   +   s/ethercat::Unexpected/std::unexpected/
// (then delete this header and include <expected>).
//
// SUBSET ONLY -- exactly what the FSM API needs: trivially-copyable T/E, no
// exceptions, no allocation, no monadic ops, everything constexpr/noexcept.
// CONTRACT (mirrors std::expected's preconditions): error() is valid ONLY when
// !has_value(); value()/operator* only when has_value(). Violations are
// debug-asserted and undefined in release -- same as std::expected.

#include <cassert>

namespace ethercat {

// std::unexpected stand-in: wraps an error for Expected's converting constructor.
template <class E>
struct Unexpected {
    E err;
    constexpr explicit Unexpected(E e) noexcept : err(e) {}
};
template <class E>
Unexpected(E) -> Unexpected<E>;

template <class T, class E>
class Expected {
   public:
    // Implicit converting ctors mirror std::expected's (value / unexpected).
    // NOLINTNEXTLINE(google-explicit-constructor,hicpp-explicit-conversions)
    constexpr Expected(T v) noexcept : value_(v), has_(true) {}
    // NOLINTNEXTLINE(google-explicit-constructor,hicpp-explicit-conversions)
    constexpr Expected(Unexpected<E> u) noexcept : error_(u.err), has_(false) {}

    [[nodiscard]] constexpr bool has_value() const noexcept {
        return has_;
    }
    constexpr explicit operator bool() const noexcept {
        return has_;
    }
    [[nodiscard]] constexpr T value() const noexcept {
        assert(has_);
        return value_;
    }
    constexpr T operator*() const noexcept {
        assert(has_);
        return value_;
    }
    [[nodiscard]] constexpr E error() const noexcept {
        assert(!has_);
        return error_;
    }

   private:
    T value_{};  // both members stored plainly (trivial payloads; no union/variant)
    E error_{};
    bool has_;
};

// void specialization: success carries no value (`return {}` constructs it).
template <class E>
class Expected<void, E> {
   public:
    constexpr Expected() noexcept = default;
    // NOLINTNEXTLINE(google-explicit-constructor,hicpp-explicit-conversions)
    constexpr Expected(Unexpected<E> u) noexcept : error_(u.err), has_(false) {}

    [[nodiscard]] constexpr bool has_value() const noexcept {
        return has_;
    }
    constexpr explicit operator bool() const noexcept {
        return has_;
    }
    [[nodiscard]] constexpr E error() const noexcept {
        assert(!has_);
        return error_;
    }

   private:
    E error_{};
    bool has_ = true;
};

}  // namespace ethercat
