#include <algorithm>
#include <iostream>
#include <iterator>
#include <vector>
 
template<class ForwardIt, class T, class Compare=std::less<>>
ForwardIt binary_find(ForwardIt first, ForwardIt last, const T& value, Compare comp={})
{
    // Note: BOTH type T and the type after ForwardIt is dereferenced 
    // must be implicitly convertible to BOTH Type1 and Type2, used in Compare. 
    // This is stricter than lower_bound requirement (see above)
 
    first = std::lower_bound(first, last, value, comp);
    return first != last && !comp(value, *first) ? first : last;
}
 
int main()
{
    std::vector<int> data = { 1, 1, 2, 3, 3, 3, 3, 4, 5, 5, 5, 5, 23 ,7 ,8};
 
    auto lower = std::lower_bound(data.begin(), data.end(), 4);
    auto upper = std::upper_bound(data.begin(), data.end(), 5);
    std::cout << *lower << "," << *upper << std::endl;
 
    std::copy(lower, upper, std::ostream_iterator<int>(std::cout, " "));
 
    std::cout << '\n';
 
    // classic binary search, returning a value only if it is present
 
    data = { 1, 2, 4, 6, 9, 10 }; 
    std::vector<int> bbb;
    int aaa = bbb.back();
 
    auto it = binary_find(data.cbegin(), data.cend(), 4); //< choosing '5' will return end()
 
    if(it != data.cend())
      std::cout << *it << " found at index "<< std::distance(data.cbegin(), it);
 
    return 0;
}
