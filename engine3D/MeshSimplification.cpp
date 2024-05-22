#include "MeshSimplification.h"

MeshSimplification::MeshSimplification(list<OBJIndex> OBJIndices, vector<vec3> vertices, float ratio) : m_vertices(vertices), m_OBJIndices(OBJIndices)
{

	readImportantVertices("./res/coords.txt");
	int originalFaceCount = m_OBJIndices.size() / 3;
	MAX_FACES = static_cast<int>(originalFaceCount * ratio);

	// Init edgeVector.
	initEdgeVector();
	
	// Init neighbors multimap.
	initVertexNeighbor();

	for (int i = 0; i < m_vertices.size(); i++) 
	{
		m_errors.push_back(calcVertexError(i));
	}
	for (int i = 0; i < m_edgeVector.size(); i++) 
	{
		calcEdgeError(m_edgeVector[i]);
	}

	buildHeap();

	// Start the simplification algorithm.
	start();
}

// Function to read the important vertices from a file.
void MeshSimplification::readImportantVertices(const string& filename) {
	ifstream file(filename);
	int index;
	while (file >> index) {
		m_importantVertices.insert(index);
	}
}

/**
* @tbrief Check if the 2 given vertices are neigbhors, they are both connected to the same vertex so combine to a triangle.
*
* @param second The index of the possible triangle.
* @param third The index of the possible triangle.
*/
bool MeshSimplification::isTriangle(int second, int third) 
{
	//second & third already in first
	pair <multimap<int, int>::iterator, multimap<int, int>::iterator> result = m_vertexNeighbor.equal_range(second);
	for (multimap<int, int>::iterator it = result.first; it != result.second; it++) 
	{
		if (third == it->second)
		{
			return true;
		}
	}
	return false;
}

/**
* @tbrief Given the indices of the edge being removed, remove the faces that are invalid now.
*
* @param firstEdgeInd The first index of the edge being removed.
* @param secondEdgeInd The second index of the edge being removed.
*/
void MeshSimplification::calcFaces(int firstEdgeInd, int secondEdgeInd)
{
	int counter = 0;
	bool setIteratorBack = false;
	OBJIndex first, second, third;
	for (list<OBJIndex>::iterator it = m_OBJIndices.begin(); it != m_OBJIndices.end(); ++it)
	{
		if (setIteratorBack)
		{
			it--;
			setIteratorBack = false;
		}

		switch (counter)
		{
		// First vertex of the current face.
		case(0) :
			first = *it;
			break;

		// Second vertex of the current face.
		case(1):
			second = *it;
			break;
				
		// Third vertex of the current face.
		case(2):
			third = *it;
			counter = 0;

			// If the current face contains the given edge indices.
			if ((firstEdgeInd == first.vertexIndex || firstEdgeInd == second.vertexIndex || firstEdgeInd == third.vertexIndex) && (secondEdgeInd == first.vertexIndex || secondEdgeInd == second.vertexIndex || secondEdgeInd == third.vertexIndex))
			{
				for (int i = 0; i < m_edgeVector.size(); )
				{
					unsigned int currEdgeFirstVertexIdx = m_edgeVector[i].firstVertex.vertexIndex;
					unsigned int currEdgeSecondVertexIdx = m_edgeVector[i].secondVertex.vertexIndex;

					// If the current edge is a part of the current face and is not connected to the firstEdgeInd then remove it.
					if ((currEdgeFirstVertexIdx != firstEdgeInd && currEdgeSecondVertexIdx != firstEdgeInd) && (currEdgeFirstVertexIdx == first.vertexIndex || currEdgeFirstVertexIdx == second.vertexIndex || currEdgeFirstVertexIdx == third.vertexIndex) && (currEdgeSecondVertexIdx == first.vertexIndex || currEdgeSecondVertexIdx == second.vertexIndex || currEdgeSecondVertexIdx == third.vertexIndex))
					{
						m_edgeVector.erase(m_edgeVector.begin() + i);
					}
					else
					{
						i++;
					}
				}
				it--;
				it--;
				m_OBJIndices.erase(it++);
				m_OBJIndices.erase(it++);
				m_OBJIndices.erase(it++);

				if (it != m_OBJIndices.begin())
					it--;
				else
					setIteratorBack = true;
			}
			continue;
		}
		counter++;
	}

	// Update m_OBJIndices that all indices that were of the removed index and set to the newly combined vertex.
	for (list<OBJIndex>::iterator it = m_OBJIndices.begin(); it != m_OBJIndices.end(); ++it)
	{
		if (it->vertexIndex == secondEdgeInd)
		{
			it->vertexIndex = firstEdgeInd;
		}
	}
}

// @tbrief Helper function to remove duplicated edges from the EdgeVector.
vector<Edge> MeshSimplification::removeDups(vector<Edge> toRemove)
{
	unsigned int i;
	unsigned int j;
	vector<Edge> noDup;
	bool found = false;
	for (i = 0; i < toRemove.size(); i++) 
	{
		found = false;
		for (j = i + 1; j < toRemove.size() && !found; j++) 
		{
			if (toRemove[i] == toRemove[j])
				found = true;
		}
		if (!found)
			noDup.push_back(toRemove[i]);
	}
	return noDup;
}

// @tbrief Initialize EdgeVector without duplicate Edges.
void MeshSimplification::initEdgeVector()
{
	int counter = 0;
	OBJIndex first;
	OBJIndex second;
	OBJIndex third;
	for (list<OBJIndex>::iterator it = m_OBJIndices.begin(); it != m_OBJIndices.end(); it++)
	{
		if (counter == 0)
			first = *it;
		if (counter == 1)
			second = *it;
		if (counter == 2)
		{
			third = *it;
			counter = 0;
			struct Edge edge1;
			struct Edge edge2;
			struct Edge edge3;

			edge1.firstVertex = first;
			edge1.secondVertex = second;

			if (first.vertexIndex > second.vertexIndex)
			{
				edge1.firstVertex = second;
				edge1.secondVertex = first;
			}

			edge2.firstVertex = first;
			edge2.secondVertex = third;

			if (first.vertexIndex > third.vertexIndex)
			{
				edge1.firstVertex = third;
				edge1.secondVertex = first;
			}
			edge3.firstVertex = second;
			edge3.secondVertex = third;

			if (second.vertexIndex > third.vertexIndex)
			{
				edge1.firstVertex = third;
				edge1.secondVertex = second;
			}
			m_edgeVector.push_back(edge1);
			m_edgeVector.push_back(edge2);
			m_edgeVector.push_back(edge3);
			continue;
		}
		counter++;
	}
	m_edgeVector = removeDups(m_edgeVector);
}

// @tbrief Initialize m_vertexNeighbor multimap.
void  MeshSimplification::initVertexNeighbor()
{
	for (size_t i = 0; i < m_vertices.size(); i++)
	{
		for (size_t j = 0; j < m_edgeVector.size(); j++)
		{
			if (i == m_edgeVector[j].firstVertex.vertexIndex)
				m_vertexNeighbor.insert(pair<int, int>(i, m_edgeVector[j].secondVertex.vertexIndex));
			if (i == m_edgeVector[j].secondVertex.vertexIndex)
				m_vertexNeighbor.insert(pair<int, int>(i, m_edgeVector[j].firstVertex.vertexIndex));
		}
	}
}

// @tbrief calculates the Quadric Error of a vertex
mat4 MeshSimplification::calcVertexError(int vertexIndex)
{
	mat4 qMat;
	pair <multimap<int, int>::iterator, multimap<int, int>::iterator> result = m_vertexNeighbor.equal_range(vertexIndex);
	// Find all the triangles of this vertex and add it's error to it.
	for (multimap<int, int>::iterator it = result.first; it != result.second; it++)
	{
		for (multimap<int, int>::iterator it2 = it; it2 != result.second; it2++)
		{
			if (it2->second != it->second)
			{
				// Up until here we check if the 3 indexes creates a triangle.
				if (isTriangle(it->second, it2->second))
				{
					// Calc cross prod.
					vec3 n = cross(m_vertices[it2->second] - m_vertices[vertexIndex], m_vertices[it->second] - m_vertices[vertexIndex]);
					n = normalize(n);

					vec4 v_tag = vec4(n, -(dot(m_vertices[vertexIndex], n)));
					for (int i = 0; i < 4; i++) 
					{
						for (int j = 0; j < 4; j++) 
						{
							qMat[i][j] += v_tag[i] * v_tag[j];
						}
					}
				}
			}
		}
	}
	return qMat;
}

void MeshSimplification::buildHeap() 
{
	make_heap(m_edgeVector.begin(), m_edgeVector.end(), compEdgeErr());
}

// @tbrief Calculate the given error of this edge.
void MeshSimplification::calcEdgeError(struct Edge& e)
{
	e.edgeQ = m_errors[e.firstVertex.vertexIndex] + m_errors[e.secondVertex.vertexIndex];

	// calc new position in the middle
	vec4 midVec = vec4((m_vertices[e.firstVertex.vertexIndex] + m_vertices[e.secondVertex.vertexIndex]) / 2.0f, 1.0);
	e.newPos = vec3(midVec);

	// temp is a row vector
	vec4 temp = e.edgeQ * midVec; 
	e.edgeError = dot(midVec, temp);
}

//  @tbrief The simplification algorithm.
void MeshSimplification::start() {
	int currFaces = m_OBJIndices.size() / 3;

	while (currFaces > MAX_FACES) {
		if (m_edgeVector.empty()) {
			std::cerr << "Error: Edge vector is empty. Cannot continue simplification." << std::endl;
			break;
		}

		// Get the edge to remove.
		pop_heap(m_edgeVector.begin(), m_edgeVector.end(), compEdgeErr());

		if (m_edgeVector.empty()) {
			std::cerr << "Error: Edge vector is empty after pop_heap. Cannot continue simplification." << std::endl;
			break;
		}
		Edge removedEdge = m_edgeVector.back();

		if (m_importantVertices.find(removedEdge.firstVertex.vertexIndex) != m_importantVertices.end() ||
			m_importantVertices.find(removedEdge.secondVertex.vertexIndex) != m_importantVertices.end()) {
			m_edgeVector.pop_back();
			continue;
		}

		int firstVertexInd = removedEdge.firstVertex.vertexIndex;
		int secondVertexInd = removedEdge.secondVertex.vertexIndex;
		vec3 newVertex = removedEdge.newPos;

		m_errors[firstVertexInd] = removedEdge.edgeQ;
		m_vertices[firstVertexInd] = newVertex;

		bool neighborToBothVertices = false;
		auto ret = m_vertexNeighbor.equal_range(secondVertexInd);
		for (auto it = ret.first; it != ret.second; ++it) {
			neighborToBothVertices = false;
			auto neighbor = m_vertexNeighbor.equal_range(it->second);
			for (auto it2 = neighbor.first; it2 != neighbor.second; ++it2) {
				if (it2->second == firstVertexInd) {
					neighborToBothVertices = true;
				}
			}
			if (it->second != firstVertexInd && !neighborToBothVertices) {
				m_vertexNeighbor.insert({ it->second, firstVertexInd });
			}
		}

		m_edgeVector.pop_back();
		currFaces -= 2;
		m_vertexNeighbor.erase(secondVertexInd);

		calcFaces(firstVertexInd, secondVertexInd);

		for (int i = 0; i < m_edgeVector.size(); i++) {
			if (m_edgeVector[i].secondVertex.vertexIndex == secondVertexInd) {
				m_edgeVector[i].secondVertex.vertexIndex = firstVertexInd;
			}
			else if (m_edgeVector[i].firstVertex.vertexIndex == secondVertexInd) {
				m_edgeVector[i].firstVertex.vertexIndex = firstVertexInd;
			}
			if (m_edgeVector[i].firstVertex.vertexIndex == firstVertexInd || m_edgeVector[i].secondVertex.vertexIndex == firstVertexInd) {
				calcEdgeError(m_edgeVector[i]);
			}
		}
		buildHeap();
	}
}


vector<vec3> MeshSimplification::getVertices()
{
	return m_vertices;
}

list<OBJIndex> MeshSimplification::getIndices()
{
	return m_OBJIndices;
}

// @tbrief Prints vector of Edges.
void MeshSimplification::printEdgeVector(vector<Edge> vec)
{
	cout << "printEdgeVector: edge vector size = " << m_edgeVector.size() << endl;
	for (unsigned int i = 0; i < vec.size(); i++)
	{
		cout << "edge[" << i << "] = (" << vec[i].firstVertex.vertexIndex << ", " << vec[i].secondVertex.vertexIndex << ") edgeError= " << vec[i].edgeError << endl;
	}
}

// @tbrief Print all the current faces from the m_OBJIndices.
void MeshSimplification::printFaces()
{
	cout << "printFaces: face list size = " << m_OBJIndices.size() / 3 << endl;

	int counter = 0;
	OBJIndex first;
	OBJIndex second;
	OBJIndex third;
	for (list<OBJIndex>::iterator it = m_OBJIndices.begin(); it != m_OBJIndices.end(); it++)
	{
		if (counter == 0)
			first = *it;
		if (counter == 1)
			second = *it;
		if (counter == 2)
		{
			third = *it;
			counter = 0;
			cout << "[" << first.vertexIndex << ", " << second.vertexIndex << ", " << third.vertexIndex << "]" << endl;
			continue;
		}
		counter++;
	}
}

// @tbrief Print m_vertexNeighbor, the multimap of indicies to it's neighbors. 
void MeshSimplification::printNeighbors()
{
	for (int vertexIdx = 0; vertexIdx < m_vertices.size(); vertexIdx++)
	{
		pair <multimap<int, int>::iterator, multimap<int, int>::iterator> ret;
		ret = m_vertexNeighbor.equal_range(vertexIdx);
		cout << "[" << vertexIdx << "]";
		for (multimap<int, int>::iterator it = ret.first; it != ret.second; it++)
		{
			cout << it->second << ", ";
		}
		cout << endl;
	}
}

MeshSimplification::~MeshSimplification()
{
}
