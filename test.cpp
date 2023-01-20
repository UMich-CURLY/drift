#include <iostream>
#include <memory>
#include <map>

using namespace std;

int main() {

    cout << "hello world" << endl;
    map<int, int> m;
    m[1] = 100;
    m[2] = 200;
    // shared_ptr<pair<int, int> > p = make_shared<pair<int, int> >(m.find(1));
    
    // create a pointer point to the first element of the map
    std::map<int, int>::iterator p = m.find(1);
    cout << p->first << " "<< p->second << endl;
    m[1] = 1000;
    cout << p->first << " "<< p->second << endl;

    shared_ptr<int> sp;
    int v = 101;
    sp = make_shared<int>(v);
    cout << *sp << " " << sp << " " << &v << endl;
    v = 102;
    cout << *sp << " " << sp << " " << &v << endl;
    return 0;
}