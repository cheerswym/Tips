/*
 * vector_front_back.cxx
 * 
 * Copyright 2019 jd <jd@jd>
 * 
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 * 
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 * 
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston,
 * MA 02110-1301, USA.
 * 
 * v: [ 1 | 2 | 3 | 4 | ... | 999 ]
     🡑                      🡑     🡑
   front()                back() end()
     🡑
   begin()
 */


#include <iostream>
#include <vector>
using namespace std;

int main()
{
 vector<char> v1;
 vector<char>::iterator iter1;
 vector<char>::iterator iter2;
 v1.push_back('m');
 v1.push_back('n');
 v1.push_back('o');
 v1.push_back('p');
 
 cout << "v1.front() = " << v1.front() << endl;
 cout << "v1.back() = " << v1.back() << endl;

 

 iter1 = v1.begin();
 cout << *iter1 << endl;
 iter2 = v1.end()-1;    //注意v1.end()指向的是最后一个元素的下一个位置，所以访问最后一个元素

                        //的正确操作为：v1.end() - 1;
 cout << *iter2 << endl;
 return 0;
}


