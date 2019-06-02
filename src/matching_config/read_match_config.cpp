//
// Created by eirik on 13.03.19.
// Acknowlegement: copied from or inspired by https://www.walletfox.com/course/parseconfigfile.php

/*
The code below parses the matching_config file. After we successfully load the file, we read it line by line.
We remove all the whitespaces from the line and skip the line if it is empty or contains a comment
(indicated by "#"). After that, we split the string "name=value" at the delimiter "=" and print the
name and the value.
*/

#include <iostream>
#include <fstream>
#include <algorithm>
#include <dirent.h>
#include <string.h>

#include <yaml-cpp/yaml.h>

using namespace std;

int countLines(const char *filename){

    int count = 0;
    string line;

    ifstream file(filename);

    while (getline(file, line)) {
        count++;
    }
    return count;
}


string readLine(const char* filename, int i) {
    ifstream cFile (filename);
    string line;
    int numLines = countLines(filename);

    if (numLines<i){
        cout << "The wanted line number doesnt exist." << endl;
    }
    else {
        for (int lineno = 0; getline(cFile, line) && lineno < numLines; lineno++)
            if (lineno == i){
                if(line[0] == '#' || line.empty()) {
                    continue;
                }
                return line;
        }
    }
}

string getName(const char* filename, int i){

    string line = readLine(filename,i);
    line.erase(remove_if(line.begin(), line.end(), ::isspace),
               line.end());
    auto delimiterPos = line.find(":");
    string name = line.substr(0, delimiterPos);
    return name;
}

string getValue(const char *filename,int i){

    string line = readLine(filename,i);
    line.erase(remove_if(line.begin(), line.end(), ::isspace),
               line.end());
    auto delimiterPos = line.find(":");
    string value = line.substr(delimiterPos + 1);
    return value;
}

void printAllLines(const char *filename){
    int numLines = countLines(filename);
    for (int i = 0; i <= numLines; i++){
        cout << readLine(filename,i) << endl;
    }
}

char* findFilesDir(const char* directoryPath, const char* fileType, int fileNumber) {
    DIR *d;
    char *p1,*p2;
    int ret;
    struct dirent *dir;
    d = opendir(directoryPath);
    int n = 0;
    if (d){
        while ((dir = readdir(d)) != NULL){
            p1=strtok(dir->d_name,".");
            p2=strtok(NULL,".");
            if(p2!=NULL){
                ret=strcmp(p2,fileType);
                if(ret==0){
                    n++;
                    if (n == fileNumber){
                        return p1;
                    }
                }
            }
        }

    closedir(d);
    }
    return(0);
}

int listFilesByType(const char* directoryPath, const char* fileType) {
    DIR *d;
    char *p1, *p2;
    int ret;
    struct dirent *dir;
    d = opendir(directoryPath);
    int n = 0;

    cout << "Files in the path " << directoryPath << " of filetype " << fileType << endl << endl;

    if (d) {
        while ((dir = readdir(d)) != NULL) {
            p1 = strtok(dir->d_name, ".");
            p2 = strtok(NULL, ".");
            if (p2 != NULL) {
                ret = strcmp(p2, fileType);
                if (ret == 0) {
                    cout << n+1 << ": " << p1 << "." << fileType << endl;
                    n++;
                }
            }
        }
        closedir(d);
    }
    return (0);
}

const char* setConfigFile(const char* filePath, const char* fileType){
    listFilesByType(filePath, fileType);

    cout << endl;
    cout << "Select config file by number: ";
    int n;
    cin >> n;
    const char * file = findFilesDir(filePath, fileType, n);

    cout << filePath << "/" << file << "." << fileType << endl;
}