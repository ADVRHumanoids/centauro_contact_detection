#include <iostream>
#include <algorithm>
#include <deque>

int main()
{
    std::cout<<"Hello World\n";

    bool contacts_flag[4][5] = {{false, false, false, false, true}, {false, false, false, false, false}, {false, false, false, false, false}, {false, false, false, false, false}};
    bool contacts[4] = {false, false, false, false};

    bool a[2] = {true, false};
    //std::cout<<a[1];

    int level = std::distance(contacts_flag[0], std::find(std::begin(contacts_flag[0]), std::end(contacts_flag[0]), true));

    std::cout<<level<<"\n";

    std::deque<bool> vect;

    vect.assign(20, false);
    //for (int i = 0; i<4; i++)
      //  for (int j = 0; j<5; j++)
        //    {
          //      vect.push_back(false);
            //}

    for (int i = 0; i<20; i++)
        {
            std::cout<<vect.at(2);
        }

    return 0;
}
