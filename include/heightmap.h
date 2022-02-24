#pragma once
#define _CRT_SECURE_NO_WARNINGS

#include <algorithm>
#include <ctime>
#include <fstream>
#include <iostream>
#include <map>
#include <memory>
#include <random>
#include <sstream>
#include <string>
#include <vector>

#include <ros/ros.h>
#include <sensor_msgs/PointCloud.h>

namespace quadtree
{
    class Point2D
    {
    public:
        Point2D() : mX(0), mZ(0) {}
        Point2D(float x, float z) : mX(x), mZ(z) {}

        float GetX() const { return mX; }
        float GetZ() const { return mZ; }

        void SetX(float x) { this->mX = x; }
        void SetZ(float z) { this->mZ = z; }

    private:
        float mX, mZ;
    };

    class Point3D
    {
    public:
        Point3D() : mX(0), mY(0), mZ(0) {}
        Point3D(float x, float y, float z) : mX(x), mY(y), mZ(z) {}

        float GetX() const { return mX; }
        float GetY() const { return mY; }
        float GetZ() const { return mZ; }
        Point2D GetEndNodeXZ() const { return mEndNodeXZ; }

        void SetX(float x) { this->mX = x; }
        void SetY(float y) { this->mY = y; }
        void SetZ(float z) { this->mZ = z; }
        void SetXYZ(float x, float y, float z) { this->mX = x; this->mY = y; this->mZ = z; }
        void SetEndNodeXZ(Point2D endNodeXZ) { this->mEndNodeXZ = endNodeXZ; }
        void SetEndNodeXZ(float x, float z) { this->mEndNodeXZ.SetX(x); this->mEndNodeXZ.SetZ(z); }

    private:
        float mX, mY, mZ;
        Point2D mEndNodeXZ;
    };

    class Boundary
    {
    public:
        Boundary() : mX(0), mZ(0), mW(0), mH(0) {}
        Boundary(float x, float z, float w, float h) : mX(x), mZ(z), mW(w), mH(h) {}
        Boundary(float minX, float maxX, float z) : mX((maxX + minX) / 2), mZ(z / 2), mW((maxX - minX) / 2), mH(z / 2) {}

        float GetX() const { return mX; }
        float GetZ() const { return mZ; }
        float GetW() const { return mW; }
        float GetH() const { return mH; }

        void SetBoundary(float x, float z, float w, float h) { this->mX = x; this->mZ = z; this->mW = w; this->mH = h; }

        bool IsConstained(Point3D* p) const { return (p->GetX() >= mX - mW && p->GetX() < mX + mW && p->GetZ() >= mZ - mH && p->GetZ() < mZ + mH); }

    private:
        float mX, mZ, mW, mH;
    };

    class Heightmap
    {
    public:
        ~Heightmap()
        {
            for (int i = 0; i < mPoints.size(); i++) {
                delete mPoints[i];
            }
            mPoints.clear();
        }
        std::vector<Point3D*> GetPoints() const { return mPoints; }

        void MakeHeightMap(Point3D* points);
        void MakeMapToVector();

    private:
        std::map<std::pair<float, float>, std::pair<float, int>> mMapPair;
        std::map<std::pair<float, float>, float> mHeightPair;
        std::vector<Point3D*> mPoints;
    };

    class Node
    {
    public:
        Node(Boundary boundary, int depth, int capacity) : mBoundary(boundary), mDepth(depth), mCapacity(capacity) {}
        Node(Boundary boundary, int depth) : mBoundary(boundary), mDepth(depth) { mCapacity = 1; }
        ~Node()
        {
            for (int i = 0; i < mPoints.size(); i++) {
                delete mPoints[i];
            }
            mPoints.clear();

            delete mHeightmap;
            mHeightmap = nullptr;
        }

        std::vector<std::unique_ptr<Point3D>> ReadPCDToVector(std::string inputPath);
        std::vector<Point3D*> SamplingPoints(std::vector<Point3D*> inputPoints, int samplingNum);
        void InsertPoints(std::vector<Point3D*> points);
        void WriteHeightmapToPCD(std::string outputPath);

        Boundary GetBoundary() const { return mBoundary; }
        void SetBoundary(Boundary boundary) { this->mBoundary = boundary; }

        Heightmap* GetHeightmap() const { return mHeightmap; }

        std::vector<Point3D*> ReadTopicToPoints(sensor_msgs::PointCloud pointcloud_msg);
        void MakeHeightmapToTopic(sensor_msgs::PointCloud& output_pointcloud);

    private:
        void subdivide();
        void insertNode(Point3D* point, Heightmap* heightmap, int depth);

        Heightmap* mHeightmap = new Heightmap();
        Boundary mBoundary;
        int mCapacity;
        int mDepth;
        bool mbDivided = false;

        std::vector<Point3D*> mPoints;
        std::vector<Point3D*> mCapacityPoints;

        std::unique_ptr<Node> mNW = nullptr;
        std::unique_ptr<Node> mNE = nullptr;
        std::unique_ptr<Node> mSW = nullptr;
        std::unique_ptr<Node> mSE = nullptr;
    };
}
