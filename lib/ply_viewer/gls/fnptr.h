#pragma once

namespace FuPtrConverter
{
template <typename Callable>
union Storage
{
    Storage() = default;
    std::decay_t<Callable> callable;
};

template <int, typename Callable, typename Ret, typename... Args>
auto FnPtr_(Callable&& c, Ret (*)(Args...))
{
    static bool used = false;
    static Storage<Callable> s;
    using type = decltype(s.callable);

    if (used)
        s.callable.~type();
    new (&s.callable) type(std::forward<Callable>(c));
    used = true;

    return [](Args... args) -> Ret {
        return Ret(s.callable(std::forward<Args>(args)...));
    };
}
} // namespace FuPtrConverter

template <typename Fn, int N = 0, typename Callable>
Fn* FnPtr(Callable&& c)
{
    return FuPtrConverter::FnPtr_<N>(std::forward<Callable>(c), (Fn*)nullptr);
}