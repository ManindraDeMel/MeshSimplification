#pragma once

#include <iostream>
#include <algorithm>
#include <glm/glm.hpp>
#include <vector>
#include <list>
#include <string>
#include <map>
#include <set>
#include <fstream>
using namespace std;
using namespace glm;

struct OBJIndex
{
	unsigned int vertexIndex;
	unsigned int uvIndex;
	unsigned int normalIndex;
	bool operator<(const OBJIndex& other) const { return vertexIndex < other.vertexIndex; }
	bool operator==(const OBJIndex& other) const { return vertexIndex == other.vertexIndex; }
};

struct Edge 
{
	OBJIndex firstVertex;
	OBJIndex secondVertex;
	float edgeError = 0;
	mat4 edgeQ;
	vec3 newPos;

	// Edge struct comperators

	bool operator==(const Edge& other) const
	{
		return (firstVertex.vertexIndex == other.firstVertex.vertexIndex && secondVertex.vertexIndex == other.secondVertex.vertexIndex) || (secondVertex.vertexIndex == other.firstVertex.vertexIndex && firstVertex.vertexIndex == other.secondVertex.vertexIndex);
	}
	bool operator()(const Edge& first, const Edge& second) const
	{
		return first < second;
	}
	bool operator < (const Edge& other)const 
	{
		if (firstVertex.vertexIndex < other.firstVertex.vertexIndex)
			return true;
		else 
		{
			if (firstVertex.vertexIndex == other.firstVertex.vertexIndex)
				return (secondVertex.vertexIndex < other.secondVertex.vertexIndex);
			else
				return false;
		}
	}
};

// Edges comperator.
struct compEdgeErr {
	bool operator()(const Edge& e1, const Edge& e2) {
		return e1.edgeError > e2.edgeError;
	}
};
class MeshSimplification {
public:
	MeshSimplification(list<OBJIndex> OBJIndices, vector<vec3> vertices, float simplificationRatio, bool CNN);
	MeshSimplification(list<OBJIndex> OBJIndices, vector<vec3> vertices, float simplificationRatio);
	~MeshSimplification();

	vector<vec3> getVertices();
	list<OBJIndex> getIndices();
	void printEdgeVector(vector<Edge> vec);
	void printFaces();
	void printNeighbors();

private:
	vector<vec3> m_vertices;
	list<OBJIndex> m_OBJIndices;
	vector<Edge> m_edgeVector;
	multimap<int, int> m_vertexNeighbor;
	vector<mat4> m_errors;
	set<pair<int, int>> m_importantEdges;
	int MAX_FACES;

	void initEdgeVector();
	void initVertexNeighbor();
	mat4 calcVertexError(int vertexIndex);
	void calcEdgeError(struct Edge& e);
	void buildHeap();
	void start();
	void startWithConsideration();
	bool isTriangle(int second, int third);
	void calcFaces(int firstEdgeInd, int secondEdgeInd);
	vector<Edge> removeDups(vector<Edge> toRemove);
	void readImportantEdges(const string& filename);
};