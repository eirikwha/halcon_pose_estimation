//
// Created by eirik on 13.03.19.
//
# pragma once
#ifndef PROJECT_READMATCHCONFIG_H
#define PROJECT_READMATCHCONFIG_H

int countLines(const char *filename);

string readLine(const char* filename, int i);

string getName(const char* filename, int i);

string getValue(const char *filename,int i);

void printAllLines(const char *filename);

char* findFilesDir(const char* directoryPath, const char* fileType, int fileNumber);

int listFilesByType(const char* directoryPath, const char* fileType);

const char* setConfigFile(const char* filePath, const char* fileType);

#endif //PROJECT_READMATCHCONFIG_H
