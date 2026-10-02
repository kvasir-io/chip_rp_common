#pragma once
// What the generated cross-check files (gen_crosscheck.py) compare: every field a pioasm
// `-o kvasir` header has, between parse()'s Program and pioasm's struct.
#include "pio/AsmParse.hpp"

#include <cstddef>

namespace crosscheck {
template<typename Ours,
         typename Theirs>
consteval bool sameWords() {
    if(Ours::Instructions.size() != Theirs::Instructions.size()) { return false; }
    for(std::size_t i = 0; i < Ours::Instructions.size(); ++i) {
        if(Ours::Instructions[i] != Theirs::Instructions[i]) { return false; }
    }
    return true;
}

template<typename Ours,
         typename Theirs>
consteval bool sameWrap() {
    return Ours::WrapTarget == Theirs::WrapTarget && Ours::Wrap == Theirs::Wrap;
}

template<typename Ours,
         typename Theirs>
consteval bool sameVersion() {
    return Ours::PioVersion == Theirs::PioVersion;
}

// isPioV1Only() over pioasm's words: a program pioasm assembled at version 0 has none
template<typename Ours,
         typename Theirs>
consteval bool v1Consistent() {
    return Theirs::PioVersion == 1 || !Kvasir::Pio::usesPioV1(Theirs::Instructions);
}

// pioasm headers carry these since kvasir_output.cpp emits them
template<typename Ours,
         typename Theirs>
consteval bool sameSideset() {
    if constexpr(requires { Theirs::SidesetCount; }) {
        return Ours::SidesetCount == Theirs::SidesetCount
            && Ours::SidesetOptional == Theirs::SidesetOptional
            && Ours::SidesetPindirs == Theirs::SidesetPindirs;
    } else {
        return true;
    }
}

template<typename Ours,
         typename Theirs>
consteval bool sameOrigin() {
    if constexpr(requires { Theirs::Origin; }) {
        return Ours::Origin == Theirs::Origin;
    } else {
        return true;
    }
}

// pioasm leaves the in/out/mov_status fields it was not given uninitialised: they are
// compared only when the directive is there. ClockDivFrac only when the text has no
// .clock_div: pioasm computes it from clock_div_frac (0) instead of the integer part, and a
// divider of 2 is then (uint8_t)512.0f.
template<typename Ours,
         typename Theirs,
         bool CompareClockDivFrac>
consteval bool sameDirectives() {
    bool ok = Ours::ClockDivInt == Theirs::ClockDivInt && Ours::FifoMode == Theirs::FifoMode
           && Ours::UsedGpioRanges == Theirs::UsedGpioRanges
           && Ours::MovStatusType == Theirs::MovStatusType && Ours::SetCount == Theirs::SetCount
           && Ours::InPinCount == Theirs::InPinCount && Ours::OutPinCount == Theirs::OutPinCount;
    if constexpr(CompareClockDivFrac) { ok = ok && Ours::ClockDivFrac == Theirs::ClockDivFrac; }
    if(Theirs::MovStatusType != -1) { ok = ok && Ours::MovStatusN == Theirs::MovStatusN; }
    if(Theirs::InPinCount != -1) {
        ok = ok && Ours::InRight == Theirs::InRight && Ours::InAutoP == Theirs::InAutoP
          && Ours::InThreshold == Theirs::InThreshold;
    }
    if(Theirs::OutPinCount != -1) {
        ok = ok && Ours::OutRight == Theirs::OutRight && Ours::OutAutoP == Theirs::OutAutoP
          && Ours::OutThreshold == Theirs::OutThreshold;
    }
    return ok;
}
}   // namespace crosscheck
