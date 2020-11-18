#ifndef IDYNTREE_SPAN_H
#define IDYNTREE_SPAN_H
///////////////////////////////////////////////////////////////////////////////
//
// Copyright (c) 2015 Microsoft Corporation. All rights reserved.
//
// This code is licensed under the MIT License (MIT).
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
// THE SOFTWARE.
//
///////////////////////////////////////////////////////////////////////////////

//Most of this file has been taken from https://github.com/Microsoft/GSL/blob/master/include/gsl/span

/*
 * Copyright (C) 2018 Fondazione Istituto Italiano di Tecnologia
 *
 * Licensed under either the GNU Lesser General Public License v3.0 :
 * https://www.gnu.org/licenses/lgpl-3.0.html
 * or the GNU Lesser General Public License v2.1 :
 * https://www.gnu.org/licenses/old-licenses/lgpl-2.1.html
 * at your option.
 */


#include <cassert>

#include <algorithm> // for lexicographical_compare
#include <array>     // for array
#include <cstddef>   // for ptrdiff_t, size_t, nullptr_t
#include <iterator>  // for reverse_iterator, distance, random_access_...
#include <limits>
#include <stdexcept>
#include <type_traits> // for enable_if_t, declval, is_convertible, inte...
#include <utility>

#ifdef _MSC_VER
#pragma warning(push)

// turn off some warnings that are noisy about our Expects statements
#pragma warning(disable : 4127) // conditional expression is constant
#pragma warning(disable : 4702) // unreachable code

// blanket turn off warnings from CppCoreCheck for now
// so people aren't annoyed by them when running the tool.
#pragma warning(disable : 26481 26482 26483 26485 26490 26491 26492 26493 26495)

#if _MSC_VER < 1910
#pragma push_macro("constexpr")
#define constexpr /*constexpr*/
#define IDYNTREE_USE_STATIC_CONSTEXPR_WORKAROUND

#endif                          // _MSC_VER < 1910
#else                           // _MSC_VER

// See if we have enough C++17 power to use a static constexpr data member
// without needing an out-of-line definition
#if !(defined(__cplusplus) && (__cplusplus >= 201703L))
#define IDYNTREE_USE_STATIC_CONSTEXPR_WORKAROUND
#endif // !(defined(__cplusplus) && (__cplusplus >= 201703L))

#endif                          // _MSC_VER

// constexpr workaround for SWIG
#ifdef SWIG
#define IDYNTREE_CONSTEXPR
#else
#define IDYNTREE_CONSTEXPR constexpr
#endif

namespace iDynTree
{

namespace SpanUtils {
#ifndef SWIG
// this is required to be compatible with c++17
template <typename... Ts> struct make_void { typedef void type; };
template <typename... Ts> using void_t = typename make_void<Ts...>::type;

//Small utility to detect if type T has value_type defined
template <typename T, typename = void> struct is_value_defined : std::false_type
{
};

template <typename T>
struct is_value_defined<T, void_t<typename T::value_type>> : std::true_type
{
};

//Small utility to detect if type T has element_type defined
template <typename T, typename = void> struct is_element_defined : std::false_type
{
};

template <typename T>
struct is_element_defined<T, void_t<typename T::element_type>> : std::true_type
{
};

//Small utility to detect if class T has the data() method defined
template <typename T, typename = void> struct has_data_method : std::false_type
{
};

template <typename T>
struct has_data_method<T, void_t<decltype(std::declval<T>().data())>> : std::true_type
{
};

//Small utility to detect if class T has the size() method defined
template <typename T, typename = void> struct has_size_method : std::false_type
{
};

template <typename T>
struct has_size_method<T, void_t<decltype(std::declval<T>().size())>> : std::true_type
{
};

#endif
}

// [views.constants], constants
IDYNTREE_CONSTEXPR const std::ptrdiff_t dynamic_extent = -1;

template <class ElementType, std::ptrdiff_t Extent = dynamic_extent>
class Span;

// implementation details
namespace details
{
    template <class T>
    struct is_span_oracle : std::false_type
    {
    };

    template <class ElementType, std::ptrdiff_t Extent>
    struct is_span_oracle<iDynTree::Span<ElementType, Extent>> : std::true_type
    {
    };

    template <class T>
    struct is_span : public is_span_oracle<std::remove_cv_t<T>>
    {
    };

    template <class T>
    struct is_std_array_oracle : std::false_type
    {
    };

    template <class ElementType, std::size_t Extent>
    struct is_std_array_oracle<std::array<ElementType, Extent>> : std::true_type
    {
    };

    template <class T>
    struct is_std_array : public is_std_array_oracle<std::remove_cv_t<T>>
    {
    };

    template <std::ptrdiff_t From, std::ptrdiff_t To>
    struct is_allowed_extent_conversion
        : public std::integral_constant<bool, From == To || From == iDynTree::dynamic_extent ||
                                                  To == iDynTree::dynamic_extent>
    {
    };

    template <class From, class To>
    struct is_allowed_element_type_conversion
        : public std::integral_constant<bool, std::is_convertible<From (*)[], To (*)[]>::value>
    {
    };

    template <class Span, bool IsConst>
    class span_iterator
    {
        using element_type_ = typename Span::element_type;

    public:

        using iterator_category = std::random_access_iterator_tag;
        using value_type = std::remove_cv_t<element_type_>;
        using difference_type = typename Span::index_type;

        using reference = std::conditional_t<IsConst, const element_type_, element_type_>&;
        using pointer = std::add_pointer_t<reference>;

        span_iterator() = default;

        IDYNTREE_CONSTEXPR span_iterator(const Span* span, typename Span::index_type idx) noexcept
            : span_(span), index_(idx)
        {}

        friend span_iterator<Span, true>;
        template<bool B, std::enable_if_t<!B && IsConst>* = nullptr>
        IDYNTREE_CONSTEXPR span_iterator(const span_iterator<Span, B>& other) noexcept
            : span_iterator(other.span_, other.index_)
        {
        }

        IDYNTREE_CONSTEXPR reference operator*() const
        {
            assert(index_ != span_->size());
            return *(span_->data() + index_);
        }

        IDYNTREE_CONSTEXPR pointer operator->() const
        {
            assert(index_ != span_->size());
            return span_->data() + index_;
        }

        IDYNTREE_CONSTEXPR span_iterator& operator++()
        {
            assert(0 <= index_ && index_ != span_->size());
            ++index_;
            return *this;
        }

        IDYNTREE_CONSTEXPR span_iterator operator++(int)
        {
            auto ret = *this;
            ++(*this);
            return ret;
        }

        IDYNTREE_CONSTEXPR span_iterator& operator--()
        {
            assert(index_ != 0 && index_ <= span_->size());
            --index_;
            return *this;
        }

        IDYNTREE_CONSTEXPR span_iterator operator--(int)
        {
            auto ret = *this;
            --(*this);
            return ret;
        }

        IDYNTREE_CONSTEXPR span_iterator operator+(difference_type n) const
        {
            auto ret = *this;
            return ret += n;
        }

        IDYNTREE_CONSTEXPR span_iterator& operator+=(difference_type n)
        {
            assert((index_ + n) >= 0 && (index_ + n) <= span_->size());
            index_ += n;
            return *this;
        }

        IDYNTREE_CONSTEXPR span_iterator operator-(difference_type n) const
        {
            auto ret = *this;
            return ret -= n;
        }

        IDYNTREE_CONSTEXPR span_iterator& operator-=(difference_type n) { return *this += -n; }

        IDYNTREE_CONSTEXPR difference_type operator-(span_iterator rhs) const
        {
            assert(span_ == rhs.span_);
            return index_ - rhs.index_;
        }

        IDYNTREE_CONSTEXPR reference operator[](difference_type n) const
        {
            return *(*this + n);
        }

       IDYNTREE_CONSTEXPR friend bool operator==(span_iterator lhs,
                                         span_iterator rhs) noexcept
        {
            return lhs.span_ == rhs.span_ && lhs.index_ == rhs.index_;
        }

        IDYNTREE_CONSTEXPR friend bool operator!=(span_iterator lhs,
                                         span_iterator rhs) noexcept
        {
            return !(lhs == rhs);
        }

        IDYNTREE_CONSTEXPR friend bool operator<(span_iterator lhs,
                                        span_iterator rhs) noexcept
        {
            return lhs.index_ < rhs.index_;
        }

        IDYNTREE_CONSTEXPR friend bool operator<=(span_iterator lhs,
                                         span_iterator rhs) noexcept
        {
            return !(rhs < lhs);
        }

        IDYNTREE_CONSTEXPR friend bool operator>(span_iterator lhs,
                                        span_iterator rhs) noexcept
        {
            return rhs < lhs;
        }

        IDYNTREE_CONSTEXPR friend bool operator>=(span_iterator lhs,
                                         span_iterator rhs) noexcept
        {
            return !(rhs > lhs);
        }

    protected:
        const Span* span_ = nullptr;
        std::ptrdiff_t index_ = 0;
    };

    template <class Span, bool IsConst>
    IDYNTREE_CONSTEXPR span_iterator<Span, IsConst>
    operator+(typename span_iterator<Span, IsConst>::difference_type n,
              span_iterator<Span, IsConst> rhs)
    {
        return rhs + n;
    }

    template <class Span, bool IsConst>
    IDYNTREE_CONSTEXPR span_iterator<Span, IsConst>
    operator-(typename span_iterator<Span, IsConst>::difference_type n,
              span_iterator<Span, IsConst> rhs)
    {
        return rhs - n;
    }

    template <std::ptrdiff_t Ext>
    class extent_type
    {
    public:
        using index_type = std::ptrdiff_t;

        static_assert(Ext >= 0, "A fixed-size span must be >= 0 in size.");

        IDYNTREE_CONSTEXPR extent_type() noexcept {}

        template <index_type Other>
        IDYNTREE_CONSTEXPR extent_type(extent_type<Other> ext)
        {
            static_assert(Other == Ext || Other == dynamic_extent,
                          "Mismatch between fixed-size extent and size of initializing data.");
            assert(ext.size() == Ext);
        }

        IDYNTREE_CONSTEXPR extent_type(index_type size) { assert(size == Ext); }

        IDYNTREE_CONSTEXPR index_type size() const noexcept { return Ext; }
    };

    template <>
    class extent_type<dynamic_extent>
    {
    public:
        using index_type = std::ptrdiff_t;

        template <index_type Other>
        explicit IDYNTREE_CONSTEXPR extent_type(extent_type<Other> ext) : size_(ext.size())
        {
        }

        explicit IDYNTREE_CONSTEXPR extent_type(index_type size) : size_(size) { assert(size >= 0); }

        IDYNTREE_CONSTEXPR index_type size() const noexcept { return size_; }

    private:
        index_type size_;
    };

    template <class ElementType, std::ptrdiff_t Extent, std::ptrdiff_t Offset, std::ptrdiff_t Count>
    struct calculate_subspan_type
    {
      using type = Span<ElementType, Count != dynamic_extent ? Count : (Extent != dynamic_extent ? Extent - Offset : Extent)>;
    };
} // namespace details

// [span], class template span
template <class ElementType, std::ptrdiff_t Extent>
class Span
{
public:
    // constants and types
    using element_type = ElementType;
    using value_type = std::remove_cv_t<ElementType>;
    using index_type = std::ptrdiff_t;
    using pointer = element_type*;
    using reference = element_type&;
    using const_reference = const element_type&;

    using iterator = details::span_iterator<Span<ElementType, Extent>, false>;
    using const_iterator = details::span_iterator<Span<ElementType, Extent>, true>;
    using reverse_iterator = std::reverse_iterator<iterator>;
    using const_reverse_iterator = std::reverse_iterator<const_iterator>;

    using size_type = index_type;

#if defined(IDYNTREE_USE_STATIC_CONSTEXPR_WORKAROUND)
    static constexpr const index_type extent { Extent };
#else
    static constexpr index_type extent { Extent };
#endif

#ifndef SWIG
    // [span.cons], span constructors, copy, assignment, and destructor
    template <bool Dependent = false,
              // "Dependent" is needed to make "std::enable_if_t<Dependent || Extent <= 0>" SFINAE,
              // since "std::enable_if_t<Extent <= 0>" is ill-formed when Extent is greater than 0.
              class = std::enable_if_t<(Dependent || Extent <= 0)>>
    IDYNTREE_CONSTEXPR Span() noexcept : storage_(nullptr, details::extent_type<0>())
    {
    }
#endif

    IDYNTREE_CONSTEXPR Span(pointer ptr, index_type count) : storage_(ptr, count) {}

    IDYNTREE_CONSTEXPR Span(pointer firstElem, pointer lastElem)
        : storage_(firstElem, std::distance(firstElem, lastElem))
    {
    }

    template <std::size_t N>
    IDYNTREE_CONSTEXPR Span(element_type (&arr)[N]) noexcept
        : storage_(KnownNotNull{&arr[0]}, details::extent_type<N>())
    {
    }

    template <std::size_t N, class ArrayElementType = std::remove_const_t<element_type>>
    IDYNTREE_CONSTEXPR Span(std::array<ArrayElementType, N>& arr) noexcept
        : storage_(&arr[0], details::extent_type<N>())
    {
    }

    template <std::size_t N>
    IDYNTREE_CONSTEXPR Span(const std::array<std::remove_const_t<element_type>, N>& arr) noexcept
        : storage_(&arr[0], details::extent_type<N>())
    {
    }

    // NB: the SFINAE here uses .data() as a incomplete/imperfect proxy for the requirement
    // on Container to be a contiguous sequence container.
#ifndef SWIG
    template <class Container,
             class = std::enable_if_t<SpanUtils::has_data_method<Container>::value && SpanUtils::has_size_method<Container>::value>,
             class = std::enable_if_t<
                 !details::is_span<Container>::value && !details::is_std_array<Container>::value &&
                 std::is_convertible<decltype(std::declval<Container>().data()), pointer>::value>>
    IDYNTREE_CONSTEXPR Span(Container& cont) : Span(cont.data(), static_cast<index_type>(cont.size()))
    {
    }

    template <class Container,
             class = std::enable_if_t<SpanUtils::has_data_method<Container>::value && SpanUtils::has_size_method<Container>::value>,
             class = std::enable_if_t<
                 std::is_const<element_type>::value && !details::is_span<Container>::value &&
                 std::is_convertible<decltype(std::declval<Container>().data()), pointer>::value>>
    IDYNTREE_CONSTEXPR Span(const Container& cont) : Span(cont.data(), static_cast<index_type>(cont.size()))
    {
    }
#endif

    IDYNTREE_CONSTEXPR Span(const Span& other) noexcept = default;

#ifndef SWIG
    template <
        class OtherElementType, std::ptrdiff_t OtherExtent,
        class = std::enable_if_t<
            details::is_allowed_extent_conversion<OtherExtent, Extent>::value &&
            details::is_allowed_element_type_conversion<OtherElementType, element_type>::value>>
    IDYNTREE_CONSTEXPR Span(const Span<OtherElementType, OtherExtent>& other)
        : storage_(other.data(), details::extent_type<OtherExtent>(other.size()))
    {
    }
#endif

    ~Span() noexcept = default;
    IDYNTREE_CONSTEXPR Span& operator=(const Span& other) noexcept = default;

    // [span.sub], span subviews
    template <std::ptrdiff_t Count>
    IDYNTREE_CONSTEXPR Span<element_type, Count> first() const
    {
        assert(Count >= 0 && Count <= size());
        return {data(), Count};
    }

    template <std::ptrdiff_t Count>
    IDYNTREE_CONSTEXPR Span<element_type, Count> last() const
    {
        assert(Count >= 0 && size() - Count >= 0);
        return {data() + (size() - Count), Count};
    }

#ifndef SWIG
    template <std::ptrdiff_t Offset, std::ptrdiff_t Count = dynamic_extent>
    IDYNTREE_CONSTEXPR auto subspan() const -> typename details::calculate_subspan_type<ElementType, Extent, Offset, Count>::type
    {
        assert((Offset >= 0 && size() - Offset >= 0) &&
                (Count == dynamic_extent || (Count >= 0 && Offset + Count <= size())));

        return {data() + Offset, Count == dynamic_extent ? size() - Offset : Count};
    }
#endif

    IDYNTREE_CONSTEXPR Span<element_type, dynamic_extent> first(index_type count) const
    {
        assert(count >= 0 && count <= size());
        return {data(), count};
    }

    IDYNTREE_CONSTEXPR Span<element_type, dynamic_extent> last(index_type count) const
    {
        return make_subspan(size() - count, dynamic_extent, subspan_selector<Extent>{});
    }

    IDYNTREE_CONSTEXPR Span<element_type, dynamic_extent> subspan(index_type offset,
                                                         index_type count = dynamic_extent) const
    {
        return make_subspan(offset, count, subspan_selector<Extent>{});
    }


    // [span.obs], span observers
    IDYNTREE_CONSTEXPR index_type size() const noexcept { return storage_.size(); }
    IDYNTREE_CONSTEXPR index_type size_bytes() const noexcept
    {
        return size() * static_cast<index_type>(sizeof(element_type));
    }
    IDYNTREE_CONSTEXPR bool empty() const noexcept { return size() == 0; }

    // [span.elem], span element access
    IDYNTREE_CONSTEXPR reference operator[](index_type idx) const
    {
        assert(idx >= 0 && idx < storage_.size());
        return data()[idx];
    }

    IDYNTREE_CONSTEXPR const_reference getVal(index_type idx) const { return this->operator[](idx);}
    IDYNTREE_CONSTEXPR bool setVal(index_type idx, const_reference val)
    {
        assert(idx >= 0 && idx < storage_.size());
        data()[idx] = val;
        return true;
    }

    IDYNTREE_CONSTEXPR reference at(index_type idx) const { return this->operator[](idx); }
    IDYNTREE_CONSTEXPR reference operator()(index_type idx) const { return this->operator[](idx); }
    IDYNTREE_CONSTEXPR pointer data() const noexcept { return storage_.data(); }

    // [span.iter], span iterator support
    IDYNTREE_CONSTEXPR iterator begin() const noexcept { return {this, 0}; }
    IDYNTREE_CONSTEXPR iterator end() const noexcept { return {this, size()}; }

    IDYNTREE_CONSTEXPR const_iterator cbegin() const noexcept { return {this, 0}; }
    IDYNTREE_CONSTEXPR const_iterator cend() const noexcept { return {this, size()}; }

    IDYNTREE_CONSTEXPR reverse_iterator rbegin() const noexcept { return reverse_iterator{end()}; }
    IDYNTREE_CONSTEXPR reverse_iterator rend() const noexcept { return reverse_iterator{begin()}; }

    IDYNTREE_CONSTEXPR const_reverse_iterator crbegin() const noexcept { return const_reverse_iterator{cend()}; }
    IDYNTREE_CONSTEXPR const_reverse_iterator crend() const noexcept { return const_reverse_iterator{cbegin()}; }

private:

    // Needed to remove unnecessary null check in subspans
    struct KnownNotNull
    {
        pointer p;
    };

    // this implementation detail class lets us take advantage of the
    // empty base class optimization to pay for only storage of a single
    // pointer in the case of fixed-size spans
    template <class ExtentType>
    class storage_type : public ExtentType
    {
    public:
        // KnownNotNull parameter is needed to remove unnecessary null check
        // in subspans and constructors from arrays
        template <class OtherExtentType>
        IDYNTREE_CONSTEXPR storage_type(KnownNotNull data, OtherExtentType ext) : ExtentType(ext), data_(data.p)
        {
             assert(ExtentType::size() >= 0);
        }


        template <class OtherExtentType>
        IDYNTREE_CONSTEXPR storage_type(pointer data, OtherExtentType ext) : ExtentType(ext), data_(data)
        {
             assert(ExtentType::size() >= 0);
             assert(data || ExtentType::size() == 0);
        }

        IDYNTREE_CONSTEXPR pointer data() const noexcept { return data_; }

    private:
        pointer data_;
    };

    storage_type<details::extent_type<Extent>> storage_;

    // The rest is needed to remove unnecessary null check
    // in subspans and constructors from arrays
    IDYNTREE_CONSTEXPR Span(KnownNotNull ptr, index_type count) : storage_(ptr, count) {}

    template <std::ptrdiff_t CallerExtent>
    class subspan_selector {};

    template <std::ptrdiff_t CallerExtent>
    Span<element_type, dynamic_extent> make_subspan(index_type offset,
                                                    index_type count,
                                                    subspan_selector<CallerExtent>) const
    {
        Span<element_type, dynamic_extent> tmp(*this);
        return tmp.subspan(offset, count);
    }

    Span<element_type, dynamic_extent> make_subspan(index_type offset,
                                                    index_type count,
                                                    subspan_selector<dynamic_extent>) const
    {
        assert(offset >= 0 && size() - offset >= 0);
        if (count == dynamic_extent)
        {
            return { KnownNotNull{ data() + offset }, size() - offset };
        }

        assert(count >= 0 && size() - offset >= count);
        return { KnownNotNull{ data() + offset },  count };
    }
};

#if defined(IDYNTREE_USE_STATIC_CONSTEXPR_WORKAROUND)
template <class ElementType, std::ptrdiff_t Extent>
IDYNTREE_CONSTEXPR const typename Span<ElementType, Extent>::index_type Span<ElementType, Extent>::extent;
#endif


// [span.comparison], span comparison operators
template <class ElementType, std::ptrdiff_t FirstExtent, std::ptrdiff_t SecondExtent>
IDYNTREE_CONSTEXPR bool operator==(Span<ElementType, FirstExtent> l,
                          Span<ElementType, SecondExtent> r)
{
    return std::equal(l.begin(), l.end(), r.begin(), r.end());
}

template <class ElementType, std::ptrdiff_t Extent>
IDYNTREE_CONSTEXPR bool operator!=(Span<ElementType, Extent> l,
                          Span<ElementType, Extent> r)
{
    return !(l == r);
}

template <class ElementType, std::ptrdiff_t Extent>
IDYNTREE_CONSTEXPR bool operator<(Span<ElementType, Extent> l,
                         Span<ElementType, Extent> r)
{
    return std::lexicographical_compare(l.begin(), l.end(), r.begin(), r.end());
}

template <class ElementType, std::ptrdiff_t Extent>
IDYNTREE_CONSTEXPR bool operator<=(Span<ElementType, Extent> l,
                          Span<ElementType, Extent> r)
{
    return !(l > r);
}

template <class ElementType, std::ptrdiff_t Extent>
IDYNTREE_CONSTEXPR bool operator>(Span<ElementType, Extent> l,
                         Span<ElementType, Extent> r)
{
    return r < l;
}

template <class ElementType, std::ptrdiff_t Extent>
IDYNTREE_CONSTEXPR bool operator>=(Span<ElementType, Extent> l,
                          Span<ElementType, Extent> r)
{
    return !(l < r);
}

namespace details
{
    // if we only supported compilers with good constexpr support then
    // this pair of classes could collapse down to a constexpr function

    // we should use a narrow_cast<> to go to std::size_t, but older compilers may not see it as
    // constexpr
    // and so will fail compilation of the template
#ifndef SWIG
    template <class ElementType, std::ptrdiff_t Extent>
    struct calculate_byte_size
        : std::integral_constant<std::ptrdiff_t,
                                 static_cast<std::ptrdiff_t>(sizeof(ElementType) *
                                                             static_cast<std::size_t>(Extent))>
    {
    };

    template <class ElementType>
    struct calculate_byte_size<ElementType, dynamic_extent>
        : std::integral_constant<std::ptrdiff_t, dynamic_extent>
    {
    };
#endif
}


#ifndef SWIG
//
// make_span() - Utility functions for creating spans
//
template <class ElementType>
IDYNTREE_CONSTEXPR Span<ElementType> make_span(ElementType* ptr, typename Span<ElementType>::index_type count)
{
    return Span<ElementType>(ptr, count);
}

template <class ElementType>
IDYNTREE_CONSTEXPR Span<ElementType> make_span(ElementType* firstElem, ElementType* lastElem)
{
    return Span<ElementType>(firstElem, lastElem);
}

template <class ElementType, std::size_t N>
IDYNTREE_CONSTEXPR Span<ElementType, N> make_span(ElementType (&arr)[N]) noexcept
{
    return Span<ElementType, N>(arr);
}

template <class Container>
IDYNTREE_CONSTEXPR Span<typename Container::value_type> make_span(Container& cont)
{
    return Span<typename Container::value_type>(cont);
}

template <class Container, typename = typename std::enable_if<SpanUtils::is_value_defined<Container>::value>::type>
IDYNTREE_CONSTEXPR Span<const typename Container::value_type> make_span(const Container& cont)
{
    return Span<const typename Container::value_type>(cont);
}

template <class Ptr>
IDYNTREE_CONSTEXPR Span<typename Ptr::element_type> make_span(Ptr& cont, std::ptrdiff_t count)
{
    return Span<typename Ptr::element_type>(cont, count);
}

template <class Ptr, typename = typename std::enable_if<!SpanUtils::is_value_defined<Ptr>::value && SpanUtils::is_element_defined<Ptr>::value>::type>
IDYNTREE_CONSTEXPR Span<typename Ptr::element_type> make_span(Ptr& cont)
{
    return Span<typename Ptr::element_type>(cont);
}

template <class Container, typename = typename std::enable_if<!SpanUtils::is_value_defined<Container>::value &&
                                                             !SpanUtils::is_element_defined<Container>::value &&
                                                             SpanUtils::has_data_method<Container>::value>::type>
IDYNTREE_CONSTEXPR Span<typename std::remove_pointer<decltype (std::declval<Container>().data())>::type> make_span(Container& cont)
{
    return Span<typename std::remove_pointer<decltype (std::declval<Container>().data())>::type>(cont);
}

#endif

} // namespace iDynTree

#ifdef _MSC_VER
#if _MSC_VER < 1910
#undef constexpr
#pragma pop_macro("constexpr")

#endif // _MSC_VER < 1910

#pragma warning(pop)
#endif // _MSC_VER

#endif // IDYNTREE_SPAN_H
