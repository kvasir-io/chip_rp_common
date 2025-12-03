#pragma once

#include "cdcacm.hpp"

#include <array>
#include <cstddef>
#include <kvasir/Atomic/Queue.hpp>
#include <utility>

namespace Kvasir::USB::detail {

// Accessor struct for mixin traits - allows friend access to private members
struct MixinTraits {
    // Forward declaration of MixinBases
    template<typename Clock,
             typename Config,
             typename Derived,
             std::size_t BaseInterface,
             std::size_t BaseEndpoint,
             typename Indices,
             template<typename, typename, typename, std::size_t, std::size_t> class... Mixins>
    struct MixinBases;

    template<typename Clock,
             typename Config,
             typename Derived,
             template<typename,
                      typename,
                      typename,
                      std::size_t,
                      std::size_t> class... Mixins>
    static consteval auto getMixinInterfaceCounts() {
        if constexpr(sizeof...(Mixins) > 0) {
            return std::array<std::size_t, sizeof...(Mixins)>{
              Mixins<Clock, Config, Derived, 0, 0>::InterfaceCount...};
        } else {
            return std::array<std::size_t, 0>{};
        }
    }

    template<typename Clock,
             typename Config,
             typename Derived,
             template<typename,
                      typename,
                      typename,
                      std::size_t,
                      std::size_t> class... Mixins>
    static consteval std::size_t getInterfaceOffset(std::size_t mixin_index) {
        constexpr auto counts = getMixinInterfaceCounts<Clock, Config, Derived, Mixins...>();
        std::size_t    offset = 0;
        for(std::size_t i = 0; i < mixin_index; ++i) { offset += counts[i]; }
        return offset;
    }

    template<typename Clock,
             typename Config,
             typename Derived,
             template<typename,
                      typename,
                      typename,
                      std::size_t,
                      std::size_t> class... Mixins>
    static consteval auto getMixinEndpointCounts() {
        if constexpr(sizeof...(Mixins) > 0) {
            return std::array<std::size_t, sizeof...(Mixins)>{
              Mixins<Clock, Config, Derived, 0, 0>::EndpointCount...};
        } else {
            return std::array<std::size_t, 0>{};
        }
    }

    template<typename Clock,
             typename Config,
             typename Derived,
             template<typename,
                      typename,
                      typename,
                      std::size_t,
                      std::size_t> class... Mixins>
    static consteval std::size_t getEndpointOffset(std::size_t mixin_index) {
        constexpr auto counts = getMixinEndpointCounts<Clock, Config, Derived, Mixins...>();
        std::size_t    offset = 0;
        for(std::size_t i = 0; i < mixin_index; ++i) { offset += counts[i]; }
        return offset;
    }

    template<typename Clock,
             typename Config,
             typename Derived,
             template<typename,
                      typename,
                      typename,
                      std::size_t,
                      std::size_t> class... Mixins>
    static consteval std::size_t countMixinInterfaces() {
        if constexpr(sizeof...(Mixins) > 0) {
            return (Mixins<Clock, Config, Derived, 0, 0>::InterfaceCount + ... + 0);
        }
        return 0;
    }

    template<typename Clock,
             typename Config,
             typename Derived,
             template<typename,
                      typename,
                      typename,
                      std::size_t,
                      std::size_t> class... Mixins>
    static consteval std::size_t countMixinEndpoints() {
        if constexpr(sizeof...(Mixins) > 0) {
            return (Mixins<Clock, Config, Derived, 0, 0>::EndpointCount + ... + 0);
        }
        return 0;
    }

    // Helper wrapper for passing mixin packs
    template<template<typename, typename, typename, std::size_t, std::size_t> class... Mixins>
    struct MixinPack {};

    // Helper to concatenate mixin descriptors at compile-time
    template<typename Clock,
             typename Config,
             typename Derived,
             std::size_t BaseInterface,
             std::size_t BaseEndpoint,
             std::size_t... Is,
             template<typename,
                      typename,
                      typename,
                      std::size_t,
                      std::size_t> class... Mixins>
    static consteval auto assembleMixinDescriptors(std::index_sequence<Is...>,
                                                   MixinPack<Mixins...>) {
        auto extractIfPresent = []<std::size_t I,
                                   template<typename, typename, typename, std::size_t, std::size_t>
                                   class Mixin>() {
            using InstantiatedMixin = Mixin<
              Clock,
              Config,
              Derived,
              BaseInterface + MixinTraits::getInterfaceOffset<Clock, Config, Derived, Mixins...>(I),
              BaseEndpoint + MixinTraits::getEndpointOffset<Clock, Config, Derived, Mixins...>(I)>;
            return Descriptors::detail::generateArray(InstantiatedMixin::InterfaceDescriptor);
        };

        if constexpr(sizeof...(Mixins) == 0) {
            return std::array<std::byte, 0>{};
        } else {
            return Descriptors::detail::generateArray(
              extractIfPresent.template operator()<Is, Mixins>()...);
        }
    }

    // Wrapper for easier use - directly calls the index_sequence version
    template<typename Clock,
             typename Config,
             typename Derived,
             std::size_t BaseInterface,
             std::size_t BaseEndpoint,
             template<typename,
                      typename,
                      typename,
                      std::size_t,
                      std::size_t> class... Mixins>
    static consteval auto assembleMixinDescriptors() {
        return assembleMixinDescriptors<Clock, Config, Derived, BaseInterface, BaseEndpoint>(
          std::make_index_sequence<sizeof...(Mixins)>{},
          MixinPack<Mixins...>{});
    }
};

// MixinBases specialization - inherits from all mixins
template<typename Clock,
         typename Config,
         typename Derived,
         std::size_t BaseInterface,
         std::size_t BaseEndpoint,
         std::size_t... Is,
         template<typename, typename, typename, std::size_t, std::size_t> class... Mixins>
struct MixinTraits::MixinBases<Clock,
                               Config,
                               Derived,
                               BaseInterface,
                               BaseEndpoint,
                               std::index_sequence<Is...>,
                               Mixins...>
  : Mixins<Clock,
           Config,
           Derived,
           BaseInterface + MixinTraits::getInterfaceOffset<Clock, Config, Derived, Mixins...>(Is),
           BaseEndpoint
             + MixinTraits::getEndpointOffset<Clock, Config, Derived, Mixins...>(Is)>... {
    // Helper methods to dispatch to all mixins
    static void callSetupEndpoints() {
        if constexpr(sizeof...(Mixins) > 0) {
            (Mixins<Clock,
                    Config,
                    Derived,
                    BaseInterface
                      + MixinTraits::getInterfaceOffset<Clock, Config, Derived, Mixins...>(Is),
                    BaseEndpoint
                      + MixinTraits::getEndpointOffset<Clock, Config, Derived, Mixins...>(
                        Is)>::SetupEndpointsCallback(),
             ...);
        }
    }

    static bool callEndpointHandler(std::size_t epNum,
                                    bool        in) {
        if constexpr(sizeof...(Mixins) > 0) {
            return (
              Mixins<Clock,
                     Config,
                     Derived,
                     BaseInterface
                       + MixinTraits::getInterfaceOffset<Clock, Config, Derived, Mixins...>(Is),
                     BaseEndpoint
                       + MixinTraits::getEndpointOffset<Clock, Config, Derived, Mixins...>(
                         Is)>::EndpointHandlerCallback(epNum, in)
              || ...);
        }
        return false;
    }

    static void callReset() {
        if constexpr(sizeof...(Mixins) > 0) {
            (Mixins<Clock,
                    Config,
                    Derived,
                    BaseInterface
                      + MixinTraits::getInterfaceOffset<Clock, Config, Derived, Mixins...>(Is),
                    BaseEndpoint
                      + MixinTraits::getEndpointOffset<Clock, Config, Derived, Mixins...>(
                        Is)>::ResetCallback(),
             ...);
        }
    }

    static void callConfigured() {
        if constexpr(sizeof...(Mixins) > 0) {
            (Mixins<Clock,
                    Config,
                    Derived,
                    BaseInterface
                      + MixinTraits::getInterfaceOffset<Clock, Config, Derived, Mixins...>(Is),
                    BaseEndpoint
                      + MixinTraits::getEndpointOffset<Clock, Config, Derived, Mixins...>(
                        Is)>::ConfiguredCallback(),
             ...);
        }
    }

    static bool callSetupPacketRequest(SetupPacket const& pkt) {
        if constexpr(sizeof...(Mixins) > 0) {
            return (
              Mixins<Clock,
                     Config,
                     Derived,
                     BaseInterface
                       + MixinTraits::getInterfaceOffset<Clock, Config, Derived, Mixins...>(Is),
                     BaseEndpoint
                       + MixinTraits::getEndpointOffset<Clock, Config, Derived, Mixins...>(
                         Is)>::SetupPacketRequestCallback(pkt)
              || ...);
        }
        return false;
    }
};

}   // namespace Kvasir::USB::detail
