//
//  Net.cpp
//
//
//  Created by Guagnlei on 03/06/2017.
//

#include <Gravity/Net.h>
#include <algorithm>
#include <map>
#define _USE_MATH_DEFINES
#include <cmath>
#include <list>
#include <iostream>
#include <fstream>
#include <sstream>
#include <iomanip>
#include <string>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <queue>
#include <time.h>

using namespace std;

Net::Net(){
    //horton_net = nullptr;
    _clone = nullptr;
    _bags = nullptr;
}

/* returns true if an arc is already present between the given nodes */
bool Net::duplicate(int n1, int n2, int id1){
    int id2 = get_arc(n1, n2)->id;
    if (id2 < id1)
        return true;
    else return false;
}


Net* Net::clone(){
    Net *copy_net = new Net();
    Node* node = NULL;
    for (int i=0; i<nodes.size(); i++) {
        node = this->nodes[i];
        copy_net->add_node(node->clone());
    }
    Arc* arc = NULL;
    for (int i=0; i<arcs.size(); i++){
        /* ignores if the arc is a paralel line to an already existing line */
        if (duplicate(arcs[i]->src->ID, arcs[i]->dest->ID, arcs[i]->id)) {
            continue;
        }
        arc = arcs[i]->clone();
        /* Update the source and destination to the new nodes in copy_net */
        arc->src = copy_net->get_node(arc->src->_name);
        arc->dest = copy_net->get_node(arc->dest->_name);
        /* Add the new arc to the list of arcs IS THIS REALLY USEFULL ? */
        copy_net->add_arc(arc);
        /* Connects it to its source and destination */
        arc->connect();
    }
    return copy_net;
}


const bool node_compare(const Node* n1, const Node* n2) {
    return n1->fill_in > n2->fill_in;
}

void Net::add_node(Node* node){
    node->ID = (int)nodes.size();
    
  //  nodeID.insert(pair<string,Node*>(_name,node)).

    
    if (nodeID.empty())
        {cout << "empty nodeID" << endl;}
    else
        {cout << nodeID.size()<< endl;}

    
    if(!nodeID.insert(pair<string,Node*>(node->_name, node)).second){
        cerr << "ERROR: adding the same node twice!";
    }
    nodes.push_back(node);
}

Node* Net::get_node(string id){
    return nodeID.find(id)->second;
}


/* returns the arc formed by node ids n1 and n2 */
Arc* Net::get_arc(Node* n1, Node* n2){
    string src, dest, key, inv_key;
    src = n1->_name;
    dest = n2->_name;
    key.clear();
    inv_key.clear();
    key.append(src);
    inv_key.append(dest);
    key.append(",");
    inv_key.append(",");
    key.append(dest);
    inv_key.append(src);
    map<string, set<Arc*>*>::iterator it= lineID.find(key);
    if (it != lineID.end()) {
        for (auto a: *it->second){
         //   if (!a->parallel) {
                return a;
           // }
        }
    }
    it = lineID.find(inv_key);
    if (it != lineID.end()) {
        for (auto a: *it->second){
         //   if (!a->parallel) {
                return a;
           // }
        }
    }
    return nullptr;
}


/* returns the Id of the arc formed by node ids n1 and n2 */
Arc* Net::get_arc(int n1, int n2){
    string src, dest, key, inv_key;
    src = to_string(n1);
    dest = to_string(n2);
    key.clear();
    inv_key.clear();
    key.append(src);
    inv_key.append(dest);
    key.append(",");
    inv_key.append(",");
    key.append(dest);
    inv_key.append(src);
    map<string, set<Arc*>*>::iterator it= lineID.find(key);
    if (it != lineID.end()) {
        for (auto a: *it->second){
          //  if (!a->parallel) {
                return a;
            //}
        }
    }
    it = lineID.find(inv_key);
    if (it != lineID.end()) {
        for (auto a: *it->second){
            //if (!a->parallel) {
                return a;
            //}
        }
    }
    return nullptr;
}



bool Net::add_arc(Arc* a){
    bool parallel = false;
    set<Arc*>* s = NULL;
    string src, dest, key, inv_key;
    src = a->src->_name;
    dest = a->dest->_name;
    key.clear();
    inv_key.clear();
    key.append(src);
    inv_key.append(dest);
    key.append(",");
    inv_key.append(",");
    key.append(dest);
    inv_key.append(src);
    if(lineID.find(key)==lineID.end() && lineID.find(inv_key)==lineID.end()){
        s = new set<Arc*>;
        s->insert(a);
        lineID.insert(pair<string, set<Arc*>*>(key,s));
    }
    else {
        if(lineID.find(key)!=lineID.end())
            s = lineID[key];
        if(lineID.find(inv_key)!=lineID.end())
            s = lineID[inv_key];
        s->insert(a);
        cout << "\nWARNING: adding another line between same nodes!\n";
      //  a->parallel = true;
        parallel = true;
    }
    arcs.push_back(a);
    return parallel;
}


void Net::remove_arc(Arc* a){
    //arcs.erase(arcs.at(a->id));
    arcs[a->id] = nullptr;
    lineID.erase(a->src->_name+","+a->dest->_name);
}

// Reading files
char* Net::readline(FILE *input)
{
    size_t len;
    int max_line_len=1024;
    char* line = new char[max_line_len];
    if(std::fgets(line,max_line_len,input)==NULL) return NULL;
    
    while(strrchr(line,'\n') == NULL)
    {
        max_line_len *= 2;
        line = (char *)realloc(line,max_line_len);
        len = strlen(line);
        if(fgets(line+len,max_line_len-len,input) == NULL)
            break;
    }
    return line;
}

void Net::exit_input_error(int line_num){
    fprintf(stderr,"Wrong input format at line %d\n", line_num);
    exit(1);
}

// readFile: just read a matrix, nothing new!
void Net::readFile(string fn){
    auto fname = fn.c_str();
    FILE *fp = fopen(fname,"r");
    if(fp == NULL)
    {
        fprintf(stderr,"can’t open input file %s\n",fname);
        exit(1);
    }
    
    size_t max_line_len = 1024;
    char* line = new char[max_line_len];
    
    vector<vector<int>> matrix;
    int temp;
    while((line = readline(fp))!=NULL)
    {
       vector<int> row;
       stringstream linestream(line);
       while (linestream>>temp)
           row.push_back(temp);
        matrix.push_back(row);
       // cout <<matrix.size()<< endl;
    }
    rewind(fp);
    delete[] line;
    fclose(fp);
}

// populates the graph
void Net::topology(string fn){
    auto fname = fn.c_str();
    FILE *fp = fopen(fname,"r");
    if(fp == NULL)
    {
        fprintf(stderr,"can’t open input file %s\n",fname);
        exit(1);
    }
    
    size_t max_line_len = 1024;
    char* line = new char[max_line_len];
    
    vector<vector<int>> matrix;
    int temp;
    while((line = readline(fp))!=NULL)
    {
        vector<int> row;
        stringstream linestream(line);
        while (linestream>>temp)
            row.push_back(temp);
        matrix.push_back(row);
        
        cout <<matrix.size()<< endl;
    }
    rewind(fp);
    delete[] line;
    fclose(fp);
}

/*  @brief Remove node and all incident arcs from the network
 @note Does not remove the incident arcs from the list of arcs in the network!
 @return the id of the node removed
 */
string Net::remove_end_node(){
    Node* n = nodes.back();
    Node * nn = nullptr;
    string n_id = n->_name;
    for (auto a: n->branches) {
        nn = a->neighbour(n);
        nn->removeArc(a);
        for (auto aa: nn->branches){
            if (!aa->neighbour(nn)->is_connected(n)) {
                nn->fill_in--;
                assert(nn->fill_in >=0);
            }
        }
    }
    nodes.pop_back();
    return n_id;
}


void Net::get_tree_decomp_bags(){
    _bags = new vector<vector<Node*>*>();
    vector<Node*>* bag = nullptr;
    Node* n = nullptr;
    Node* u = nullptr;
    Node* nn = nullptr;
    Arc* arc = nullptr;
    int id = 0;
    int nb3 = 0;
    while (_clone->nodes.size()>2) {
        sort(_clone->nodes.begin(), _clone->nodes.end(),node_compare);
        n = _clone->nodes.back();
        cout << n->_name << endl;
        cout<<_clone->nodes.size() << endl;
        bag = new vector<Node*>();
              //  cout << "new bag = { ";
        for (auto a: n->branches) {
            nn = a->neighbour(n);
            bag->push_back(nn);
            cout << nn->_name << ", ";
        }
        _clone->remove_end_node();
        for (int i = 0; i<bag->size(); i++) {
            u = bag->at(i);
            for (int j = i+1; j<bag->size(); j++) {
                nn = bag->at(j);
                if (u->is_connected(nn)) {
                    continue;
                }
                id = (int)_clone->arcs.size() + 1;
                //arc = new Arc(u->_name+nn->_name);
                arc = new Arc(to_string(id));
                arc->id = id;
                arc->src = u;
                arc->dest = nn;
                arc->connect();
                _clone->add_arc(arc);
            }
        }
        bag->push_back(n);
        //        cout << n->_name << "}\n";
        _bags->push_back(bag);
        if (bag->size()==3) {
            nb3++;
        }
    }
       cout << "\nNumber of 3D bags = " << nb3 << endl;
}


/* Destructors */
Net::~Net(){
    if (!nodes.empty()) {
        for (vector<Node*>::iterator it = nodes.begin(); it != nodes.end(); it++){
            delete (*it);
        }
        nodes.clear();
    }
    if(!arcs.empty()){
        for (vector<Arc*>::iterator it = arcs.begin(); it != arcs.end(); it++){
            if(*it)
                delete (*it);
        }
        arcs.clear();
    }
   
    if(!cycle_basis.empty()){
        for (vector<Path*>::iterator it = cycle_basis.begin(); it != cycle_basis.end(); it++){
            delete (*it);
        }
        arcs.clear();
    }
    for (pair<string,set<Arc*>*> it:lineID) {
        delete it.second;
    }
}

int Net::test(){
    string name;
    int id = 0;
    _clone = new Net();

    Node* node = NULL;
    Node* node_clone = NULL;
   
    for (int i= 0; i<3; i++){
        name = to_string(i);
        id = i;
        cout << "name " << name << " ID: " << i << endl;
        node = new Node(name,id);
        node_clone = new Node(name,i);
        add_node(node);
        _clone->add_node(node_clone);
    }

    Arc* arc = NULL;
    Arc* arc_clone = NULL;
    string src, dest;
    for (int i= 0; i<3; i++){
        for (int j = i; j<i+1; j++)
        id = (int)arcs.size() + 1;
        //arc = new Arc(src+dest);
        //arc_clone = new Arc(src+dest);
        arc = new Arc(to_string(id));
        arc_clone = new Arc(to_string(id));
        arc->id = id;
        arc_clone->id = id;
        arc->src = get_node(src);
        arc->dest= get_node(dest);
        arc_clone->src = _clone->get_node(src);
        arc_clone->dest = _clone->get_node(dest);
        arc->connect();
        arc_clone->connect();
        _clone->add_arc(arc_clone);
    }

    get_tree_decomp_bags();
    return 0;
}
