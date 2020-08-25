#include <gravity/utils.h>
using namespace std;
using namespace gravity;

gravity::indices time(unsigned p1 ,unsigned p2){
    auto res = range(p1,p2);
    res._time_extended = true;
    return res;
}

gravity::indices gravity::range(size_t p1 ,size_t p2){
    indices res("range("+to_string(p1)+"<"+to_string(p2)+")");
    auto n = p2 - p1 + 1;
    assert(n >= 0);
    res._keys_map = make_shared<map<string,size_t>>();
    res._keys = make_shared<vector<string>>();
    res._keys->resize(n);
    res._dim = make_shared<vector<size_t>>();
    res._dim->resize(1);
    res._dim->at(0) = n;
    size_t index = 0;
    for (auto i = p1; i <= p2; i++){
        (*res._keys)[index] = to_string(i);
        (*res._keys_map)[res._keys->at(index)] = index;
        index++;
    }
    return res;
}


gravity::indices gravity::operator-(const gravity::indices& s1, const gravity::indices& s2){
    gravity::indices res;
    for(auto &key: *s1._keys){
        if(s2._keys_map->count(key)==0)
            res.add(key);
    }
    return res;
}

#ifdef _WIN32
#include <Windows.h>

double get_wall_time() {
    LARGE_INTEGER time,freq;
    if (!QueryPerformanceFrequency(&freq)) {
        return 0;
    }
    if (!QueryPerformanceCounter(&time)) {
        return 0;
    }
    return (double)time.QuadPart / freq.QuadPart;
}
double get_cpu_time() {
    FILETIME a,b,c,d;
    if (GetProcessTimes(GetCurrentProcess(),&a,&b,&c,&d) != 0) {
        return
        (double)(d.dwLowDateTime |
                 ((unsigned long long)d.dwHighDateTime << 32)) * 0.0000001;
    } else {
        return 0;
    }
}
#else
#include <time.h>
#include <sys/time.h>
double get_wall_time() {
    struct timeval time;
    if (gettimeofday(&time,NULL)) {
        return 0;
    }
    return (double)time.tv_sec + (double)time.tv_usec * .000001;
}
double get_cpu_time() {
    return (double)clock() / CLOCKS_PER_SEC;
}
#endif


string clean_print(bool pos, const string& v, bool brackets){
    if(pos){
        if (v=="-1" || v==" - 1" || v=="(-1,0)") {
            return " - ";
        }
        else if (v.front()=='-'){
            return " - " + v.substr(1);
        }
        else if(v=="1" || v==" + 1" || v=="(1,0)") {
            return " + ";
        }
        else if(brackets){
            return " + ("+v+")";
        }
        else{
            return " + " + v;
        }
    }
    else {
        if (v == "-1" || v==" - 1" || v=="(-1,0)") {
            return " + ";
        }
        else if (v.front()=='-'){
            return " + " + v.substr(1);
        }
        else if (v=="1" || v==" + 1" || v=="(1,0)"){
            return " - ";
        }
        else if(brackets){
            return " - ("+v+")";
        }
        else{
            return " - " + v;
        }
    }
}

//int gravity::nthOccurrence(const std::string& str, const std::string& findMe, int nth)
//{
//    size_t  pos = 0;
//    int     cnt = 0;
//    
//    while( cnt != nth )
//    {
//        pos+=1;
//        pos = str.find(findMe, pos);
//        if ( pos == std::string::npos )
//            return -1;
//        cnt++;
//    }
//    return pos;
//}
#ifdef USE_OPT_PARSER
op::OptionParser readOptions(int argc, char * argv[]){
    string log_level ="0";
    op::OptionParser opt;
    opt.add_option("h", "help", "shows option help"); // no default value means boolean options, which default value is false
    opt.add_option("l", "log", "Log level (def. 0)", log_level );
    
    return opt;
}
#endif

bool operator <(const Cpx& lhs, const Cpx& rhs){
    return lhs.real()<rhs.real() && lhs.imag()<rhs.imag();
}

bool operator >(const Cpx& lhs, const Cpx& rhs){
    return lhs.real()>rhs.real() && lhs.imag()>rhs.imag();
}

bool operator <=(const Cpx& lhs, const Cpx& rhs){
    return lhs.real()<=rhs.real() && lhs.imag()<=rhs.imag();
}

bool operator >=(const Cpx& lhs, const Cpx& rhs){
    return lhs.real()>=rhs.real() && lhs.imag()>=rhs.imag();
}




//Cpx gravity::min (const Cpx& a, const Cpx& b){
//    Cpx res(a);
//    if (res.real()>b.real()) {
//        res.real(b.real());
//    }
//    if (res.imag()>b.imag()) {
//        res.imag(b.imag());
//    }
//    return res;
//}
//
//Cpx gravity::max (const Cpx& a, const Cpx& b)
//{
//    Cpx res(a);
//    if (res.real()<b.real()) {
//        res.real(b.real());
//    }
//    if (res.imag()<b.imag()) {
//        res.imag(b.imag());
//    }
//    return res;
//}

Sign reverse(Sign s) {
    if(s==unknown_){
        return unknown_;
    }
    return Sign(-1*s);
}

Sign sign_add(Sign s1, Sign s2){
    if (s1==unknown_ || s2==unknown_) {
        return unknown_;
    }
    else if((s1==non_neg_ || s1==pos_) && (s2==neg_ || s2==non_pos_)){
        return unknown_;
    }
    else if((s1==non_pos_ || s1==neg_) && (s2==pos_ || s2==non_neg_)){
        return unknown_;
    }
    else if(s1==zero_ || s1==pos_ || s1==neg_){// take weaker sign
        return s2;
    }
    else{
        return s1;
    }
}

Sign sign_product(Sign s1, Sign s2){
    if (s1==unknown_ || s2==unknown_) {
        return unknown_;
    }
    else if(s1==pos_ && (s2==neg_ || s2==non_pos_)){
        return s2;
    }
    else if(s1==non_neg_ && (s2==neg_ || s2==non_pos_)){
        return non_pos_;
    }
    else if(s1==neg_ && s2==neg_){
        return pos_;
    }
    else if(s1==neg_ && s2==non_pos_){
        return non_neg_;
    }
    return s1;
}

/*Split "mem" into "parts", e.g. if mem = 10 and parts = 4 you will have: 0,2,4,6,10, i.e., [0,3], [3,6], [6], [6,8], [810] if possible the function will split mem into equal chuncks, if not the first few chunks will be larger by 1*/
std::vector<size_t> bounds(unsigned parts, size_t mem) {
    std::vector<size_t>bnd;
    unsigned new_parts = parts;
    if(parts>mem){
        DebugOff("In function std::vector<size_t> bounds(unsigned parts, size_t mem), parts cannot be strictly greater than mem");
        new_parts = mem;
    }
    size_t delta = mem / new_parts;
    size_t reminder = mem % new_parts;
    size_t N1 = 0, N2 = 0;
    bnd.push_back(N1);
    for (size_t i = 0; i < new_parts; ++i) {
        N2 = N1 + delta;
        if(i<reminder)
            N2+=1;
//        if (i == new_parts - 1)
//            N2 += reminder;
        bnd.push_back(N2);
        N1 = N2;
    }
//    for(size_t i=1;i<=reminder;++i)
//        bnd.at(i)+=1;
    if(bnd.back()!=mem){
        DebugOn("Error in computing limits");
    }
    return bnd;
}

/*Split "mem" into "parts", e.g. if mem = 10 and parts = 4 you will have: 0,2,4,6,10, i.e., [0,3], [3,6], [6], [6,8], [810] if possible the function will split mem into equal chuncks, if not the first few chunks will be larger by 1. Try to assign each mem back to the part it was assigned before*/
std::vector<size_t> bounds_reassign(unsigned parts, vector<string>& objective_models, map<string,int>& old_map, int nb_threads) {
    std::vector<size_t>bnd;
    std::vector<std::string> unassign, new_objective_models;
    size_t mem=objective_models.size();
    unsigned new_parts = parts;
    if(parts>mem){
        DebugOff("In function std::vector<size_t> bounds(unsigned parts, size_t mem), parts cannot be strictly greater than mem");
        new_parts = mem;
    }
    for (auto i=0;i<=new_parts;i++){
        bnd.push_back(0);
    }
    
    int wid;
    for(auto &s:objective_models){
        if(old_map.find(s)!=old_map.end()){
            wid=old_map.at(s);
            if(wid+1<bnd.size()){
            if(bnd[wid+1] < nb_threads){
                bnd[wid+1]++;
            }
            else{
                unassign.push_back(s);
            }
            }
            else{
                unassign.push_back(s);
            }
        }
        else{
        unassign.push_back(s);
        }
    }
    wid=0;
    for(auto u=unassign.begin();u<unassign.end();u++){
            if(bnd[wid+1] < nb_threads){
            old_map[*u]=wid;
            bnd[wid+1]++;
        }
            else{
                wid++;
                u--;
               // bnd[wid+1]=bnd[wid];
            }
    }

    for(auto w=bnd.begin();w<bnd.end()-1;w++){
        for(auto it=objective_models.begin();it!=objective_models.end();it++){
            if(old_map[*it]==(w - bnd.begin())){
                new_objective_models.push_back(*it);
            }
        }
    }
     for(auto w=bnd.begin()+1;w<bnd.end();w++){
         *w=*w+*(w-1);
     }
    //    for(size_t i=1;i<=reminder;++i)
    //        bnd.at(i)+=1;
    objective_models=new_objective_models;
    if(bnd.back()!=mem){
        DebugOn("Error in computing limits");
    }
    return bnd;
}
