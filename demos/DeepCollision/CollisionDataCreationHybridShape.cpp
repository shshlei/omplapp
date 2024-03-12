/*********************************************************************
* Software License Agreement (BSD License)
*
*  Copyright (c) 2008, Willow Garage, Inc.
*  All rights reserved.
*
*  Redistribution and use in source and binary forms, with or without
*  modification, are permitted provided that the following conditions
*  are met:
*
*   * Redistributions of source code must retain the above copyright
*     notice, this list of conditions and the following disclaimer.
*   * Redistributions in binary form must reproduce the above
*     copyright notice, this list of conditions and the following
*     disclaimer in the documentation and/or other materials provided
*     with the distribution.
*   * Neither the name of the Willow Garage nor the names of its
*     contributors may be used to endorse or promote products derived
*     from this software without specific prior written permission.
*
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
*  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
*  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
*  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
*  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
*  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
*  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
*  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
*  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
*  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
*  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
*  POSSIBILITY OF SUCH DAMAGE.
*********************************************************************/

/* Author: Shi Shenglei */

#include <ompl/util/RandomNumbers.h>
#include <box2d_collision/b2_bvh_manager.h>

#include <boost/math/constants/constants.hpp>
#include <boost/filesystem.hpp>
#include <boost/program_options.hpp>
#include <boost/algorithm/string.hpp>
#include <boost/format.hpp>

#include <vector>
#include <iostream>
#include <fstream>

#include "H5Cpp.h"
using namespace H5;

class MQueryCallback : public b2NaiveCallback
{
public:
    MQueryCallback(const b2Body * body) : b2NaiveCallback()
    {
        body_ = body;
    }

    ~MQueryCallback() override = default;

    bool ReportCollision(b2Fixture* fixture) override
    {
        if (fixture->GetBody() != body_)
        {
            const b2Fixture* flist = body_->GetFixtureList();
            if (b2CollideShapes(flist->GetShape(), flist->GetGlobalTransform(), fixture->GetShape(), fixture->GetGlobalTransform()))
                collisionBodies_.insert(fixture->GetBody()->GetName());
        }
        return false; // Never stop early
    }

    const std::set<std::string> & getCollisionBodies() const 
    {
        return collisionBodies_;
    }

private:
    const b2Body* body_;
    std::set<std::string> collisionBodies_; 
};

bool argParse(int argc, char** argv, std::string &save_dir, std::size_t &objects, std::size_t &points, std::size_t &expect_obstacles, std::string &shape1, std::string &shape2);

int main(int argc, char* argv[])
{
    std::size_t objects = 1000;
    std::size_t points = 1000;
    std::size_t expect_obstacles = 1000;
    std::string save_dir, shape1, shape2;

    // Parse the arguments, returns true if successful, false otherwise
    if (!argParse(argc, argv, save_dir, objects, points, expect_obstacles, shape1, shape2))
        return -1;

    b2AABB aabb;
    aabb.min().setConstant(b2Scalar(0.0));
    aabb.max().setConstant(b2Scalar(1.0));

    ompl::RNG rng;

    // Create a new file using the default property lists.
    // H5::H5F_ACC_TRUNC : create a new file or overwrite an existing file.
    H5File label_file((save_dir + "hybrid_" + shape1 + "_" + shape2 + "_" + "collision_labels.hdf5").c_str(), H5F_ACC_TRUNC);
    std::vector<double> labels; 

    int valid = 0, invalid = 0;
    int diff = std::floor(0.05 * objects * points);
    for (std::size_t i = 0; i < objects; i++)
    {
        std::cout << "Generating the " << i << "th env ...";
        b2BVHManager manager;
        for (std::size_t o = 0; o < expect_obstacles;) // add shape
        {
            b2Shape *shape = nullptr;
            double xc = rng.uniform01();
            double yc = rng.uniform01();
            double yaw = rng.uniformReal(-boost::math::constants::pi<double>(), boost::math::constants::pi<double>());
            if (shape1 == "circle") // circle shape
            {
                yaw = 0.0;
                b2CircleShape *cshape = new b2CircleShape(b2Scalar(rng.uniformReal(0.05, 0.45)));
                shape = cshape;
            }
            else if (shape1 == "ellipse") // ellipse shape
            {
                b2EllipseShape *eshape = new b2EllipseShape(b2Scalar(rng.uniformReal(0.05, 0.45)), b2Scalar(rng.uniformReal(0.05, 0.45)));
                shape = eshape;
            }
            else if (shape1 == "capsule") // capsule shape
            {
                b2CapsuleShape *cshape = new b2CapsuleShape(b2Scalar(rng.uniformReal(0.05, 0.45)), b2Scalar(rng.uniformReal(0.05, 0.45)));
                shape = cshape;
            }
            else if (shape1 == "rectangle") // rectangle shape
            {
                b2RectangleShape *rectshape = new b2RectangleShape(b2Scalar(rng.uniformReal(0.05, 0.45)), b2Scalar(rng.uniformReal(0.05, 0.45)));
                shape = rectshape;
            }
            b2Transform xf = b2Transform::Identity();
            xf.translation() = b2Vec2(b2Scalar(xc), b2Scalar(yc));
            xf.linear() = b2Rot(b2Scalar(yaw)).toRotationMatrix();
            b2AABB saabb;
            shape->ComputeAABB(&saabb, xf);
            if (aabb.contains(saabb))
            {
                o++;
                manager.AddBody("shape" + std::to_string(o), shape, xf, false);
            }
            delete shape;
        }

        const b2Body* bodylist = manager.GetBodyList();
        while (bodylist) // disjoint
        {
            MQueryCallback callback(bodylist);

            const b2Fixture *flist = bodylist->GetFixtureList();
            while (flist)
            {
                b2AABB faabb = flist->GetAABB();
                manager.Collide(&callback, faabb);
                flist = flist->GetNext();
            }

            const std::set<std::string> & collisions = callback.getCollisionBodies();
            for (auto & name : collisions)
                manager.RemoveBody(name);

            bodylist = bodylist->GetNext();
        }

        H5File e_file((save_dir + "hybrid_" + shape1 + "_" + shape2 + "_env_" + std::to_string(i) + ".hdf5").c_str(), H5F_ACC_TRUNC);
        std::cout << "Saving env ...";

        std::vector<double> circles, ellipses, capsules, rectangles;
        bodylist = manager.GetBodyList();
        while (bodylist)
        {
            const b2Fixture *flist = bodylist->GetFixtureList();
            const b2Shape *shape = flist->GetShape();
            const b2Transform& xf = flist->GetGlobalTransform();

            if (shape->GetType() == b2Shape::e_circle)
            {
                const b2CircleShape *cshape = static_cast<const b2CircleShape *>(shape);
                circles.push_back(static_cast<double>(xf.translation().x()));
                circles.push_back(static_cast<double>(xf.translation().y()));
                circles.push_back(static_cast<double>(cshape->GetRadius()));
            }
            else if (shape->GetType() == b2Shape::e_ellipse)
            {
                const b2EllipseShape *eshape = static_cast<const b2EllipseShape *>(shape);
                b2Vec2 hsides = eshape->GetHalfSides();
                ellipses.push_back(static_cast<double>(xf.translation().x()));
                ellipses.push_back(static_cast<double>(xf.translation().y()));
                double angle = static_cast<double>(b2Rot(xf.linear()).angle());
                ellipses.push_back(std::cos(angle));
                ellipses.push_back(std::sin(angle));
                ellipses.push_back(static_cast<double>(hsides.x()));
                ellipses.push_back(static_cast<double>(hsides.y()));
            }
            else if (shape->GetType() == b2Shape::e_capsule)
            {
                const b2CapsuleShape *cshape = static_cast<const b2CapsuleShape *>(shape);
                capsules.push_back(static_cast<double>(xf.translation().x()));
                capsules.push_back(static_cast<double>(xf.translation().y()));
                double angle = static_cast<double>(b2Rot(xf.linear()).angle());
                capsules.push_back(std::cos(angle));
                capsules.push_back(std::sin(angle));
                capsules.push_back(static_cast<double>(cshape->GetRadius()));
                capsules.push_back(static_cast<double>(cshape->GetHeight()));
            }
            else if (shape->GetType() == b2Shape::e_rectangle)
            {
                const b2RectangleShape *rectshape = static_cast<const b2RectangleShape *>(shape);
                b2Vec2 hsides = rectshape->GetHalfSides();
                rectangles.push_back(static_cast<double>(xf.translation().x()));
                rectangles.push_back(static_cast<double>(xf.translation().y()));
                double angle = static_cast<double>(b2Rot(xf.linear()).angle());
                rectangles.push_back(std::cos(angle));
                rectangles.push_back(std::sin(angle));
                rectangles.push_back(static_cast<double>(hsides.x()));
                rectangles.push_back(static_cast<double>(hsides.y()));
            }

            bodylist = bodylist->GetNext();
        }

        // Create a group under root '/'.
        // Group group(file.createGroup(boost::str(boost::format("scenario_%i.txt") % i).c_str()));
        if (!circles.empty())
        {
            // Use H5::hsize_t (similar to int) for dimensions.
            hsize_t dims[2];               // dataset dimensions
            dims[0] = circles.size() / 3;
            dims[1] = 3;

            // Create the dataspace for a dataset first.
            DataSpace dataspace(2, dims);

            // Create the dataset under group with specified dataspace.      
            DataSet dataset = e_file.createDataSet("circles", PredType::NATIVE_DOUBLE, dataspace);

            // Write data in buffer to dataset.
            dataset.write(circles.data(), PredType::NATIVE_DOUBLE);
        }

        if (!ellipses.empty())
        {
            // Use H5::hsize_t (similar to int) for dimensions.
            hsize_t dims[2];               // dataset dimensions
            dims[0] = ellipses.size() / 6;
            dims[1] = 6;

            // Create the dataspace for a dataset first.
            DataSpace dataspace(2, dims);

            // Create the dataset under group with specified dataspace.      
            DataSet dataset = e_file.createDataSet("ellipses", PredType::NATIVE_DOUBLE, dataspace);

            // Write data in buffer to dataset.
            dataset.write(ellipses.data(), PredType::NATIVE_DOUBLE);
        }

        if (!capsules.empty())
        {
            // Use H5::hsize_t (similar to int) for dimensions.
            hsize_t dims[2];               // dataset dimensions
            dims[0] = capsules.size() / 6;
            dims[1] = 6;

            // Create the dataspace for a dataset first.
            DataSpace dataspace(2, dims);

            // Create the dataset under group with specified dataspace.      
            DataSet dataset = e_file.createDataSet("capsules", PredType::NATIVE_DOUBLE, dataspace);

            // Write data in buffer to dataset.
            dataset.write(capsules.data(), PredType::NATIVE_DOUBLE);
        }

        if (!rectangles.empty())
        {
            // Use H5::hsize_t (similar to int) for dimensions.
            hsize_t dims[2];               // dataset dimensions
            dims[0] = rectangles.size() / 6;
            dims[1] = 6;

            // Create the dataspace for a dataset first.
            DataSpace dataspace(2, dims);

            // Create the dataset under group with specified dataspace.      
            DataSet dataset = e_file.createDataSet("rectangles", PredType::NATIVE_DOUBLE, dataspace);

            // Write data in buffer to dataset.
            dataset.write(rectangles.data(), PredType::NATIVE_DOUBLE);
        }

        { // attribute
            int attr1_data[1] = {manager.GetProxyCount()};

            hsize_t attr1_dims[1] = {1};

            // Create the dataspace for an attribute first.
            DataSpace attr1_dataspace(1, attr1_dims);

            // Create the attribute of dataset with specified dataspace.
            Attribute attribute1 = e_file.createAttribute("Obstacles", PredType::NATIVE_INT, attr1_dataspace);

            // Write data in buffer to attribute.
            attribute1.write(PredType::NATIVE_INT, attr1_data);
        }

        e_file.close();
        std::cout << "done! ...";

        std::cout << "Generating robots ...";
        for (std::size_t j = 0; j < points;)
        {
            circles.clear();
            ellipses.clear();
            capsules.clear();
            rectangles.clear();

            b2Shape *shape = nullptr;
            double xc = rng.uniform01();
            double yc = rng.uniform01();
            double yaw = rng.uniformReal(-boost::math::constants::pi<double>(), boost::math::constants::pi<double>());
            double a = 0., b = 0.;
            if (shape2 == "circle") // circle shape
            {
                yaw = 0.0;
                a = rng.uniformReal(0.05, 0.45);
                b2CircleShape *cshape = new b2CircleShape(b2Scalar(a));
                shape = cshape;
            }
            else if (shape2 == "ellipse") // ellipse shape
            {
                a = rng.uniformReal(0.05, 0.45), b = rng.uniformReal(0.05, 0.45);
                b2EllipseShape *eshape = new b2EllipseShape(b2Scalar(a), b2Scalar(b));
                shape = eshape;
            }
            else if (shape2 == "capsule") // capsule shape
            {
                a = rng.uniformReal(0.05, 0.45), b = rng.uniformReal(0.05, 0.45);
                b2CapsuleShape *cshape = new b2CapsuleShape(b2Scalar(a), b2Scalar(b));
                shape = cshape;
            }
            else if (shape2 == "rectangle") // rectangle shape
            {
                a = rng.uniformReal(0.05, 0.45), b = rng.uniformReal(0.05, 0.45);
                b2RectangleShape *rectshape = new b2RectangleShape(b2Scalar(a), b2Scalar(b));
                shape = rectshape;
            }
            b2Transform xf = b2Transform::Identity();
            xf.translation() = b2Vec2(b2Scalar(xc), b2Scalar(yc));
            xf.linear() = b2Rot(b2Scalar(yaw)).toRotationMatrix();
            manager.AddBody("robot", shape, xf, true);
            delete shape;

            bool collision = manager.Collide();
            manager.RemoveBody("robot");
            if (valid - invalid > diff && !collision)
                continue;
            if (invalid - valid > diff && collision)
                continue;

            H5File r_file((save_dir + "hybrid_" + shape1 + "_" + shape2 + "_robot_" + std::to_string(i * points + j) + ".hdf5").c_str(), H5F_ACC_TRUNC);
            if (shape2 == "circle") // circle shape
            {
                circles.push_back(static_cast<double>(xf.translation().x()));
                circles.push_back(static_cast<double>(xf.translation().y()));
                circles.push_back(a);

                // Use H5::hsize_t (similar to int) for dimensions.
                hsize_t dims[2];               // dataset dimensions
                dims[0] = circles.size() / 3;
                dims[1] = 3;

                // Create the dataspace for a dataset first.
                DataSpace dataspace(2, dims);

                // Create the dataset under group with specified dataspace.      
                DataSet dataset = r_file.createDataSet("circles", PredType::NATIVE_DOUBLE, dataspace);

                // Write data in buffer to dataset.
                dataset.write(circles.data(), PredType::NATIVE_DOUBLE);
            }
            else if (shape2 == "ellipse") // ellipse shape
            {
                ellipses.push_back(static_cast<double>(xf.translation().x()));
                ellipses.push_back(static_cast<double>(xf.translation().y()));
                double angle = static_cast<double>(b2Rot(xf.linear()).angle());
                ellipses.push_back(std::cos(angle));
                ellipses.push_back(std::sin(angle));
                ellipses.push_back(a);
                ellipses.push_back(b);

                // Use H5::hsize_t (similar to int) for dimensions.
                hsize_t dims[2];               // dataset dimensions
                dims[0] = ellipses.size() / 6;
                dims[1] = 6;

                // Create the dataspace for a dataset first.
                DataSpace dataspace(2, dims);

                // Create the dataset under group with specified dataspace.      
                DataSet dataset = r_file.createDataSet("ellipses", PredType::NATIVE_DOUBLE, dataspace);

                // Write data in buffer to dataset.
                dataset.write(ellipses.data(), PredType::NATIVE_DOUBLE);
            }
            else if (shape2 == "capsule") // capsule shape
            {
                capsules.push_back(static_cast<double>(xf.translation().x()));
                capsules.push_back(static_cast<double>(xf.translation().y()));
                double angle = static_cast<double>(b2Rot(xf.linear()).angle());
                capsules.push_back(std::cos(angle));
                capsules.push_back(std::sin(angle));
                capsules.push_back(a);
                capsules.push_back(b);

                // Use H5::hsize_t (similar to int) for dimensions.
                hsize_t dims[2];               // dataset dimensions
                dims[0] = capsules.size() / 6;
                dims[1] = 6;

                // Create the dataspace for a dataset first.
                DataSpace dataspace(2, dims);

                // Create the dataset under group with specified dataspace.      
                DataSet dataset = r_file.createDataSet("capsules", PredType::NATIVE_DOUBLE, dataspace);

                // Write data in buffer to dataset.
                dataset.write(capsules.data(), PredType::NATIVE_DOUBLE);
            }
            else if (shape2 == "rectangle") // rectangle shape
            {
                rectangles.push_back(static_cast<double>(xf.translation().x()));
                rectangles.push_back(static_cast<double>(xf.translation().y()));
                double angle = static_cast<double>(b2Rot(xf.linear()).angle());
                rectangles.push_back(std::cos(angle));
                rectangles.push_back(std::sin(angle));
                rectangles.push_back(a);
                rectangles.push_back(b);

                // Use H5::hsize_t (similar to int) for dimensions.
                hsize_t dims[2];               // dataset dimensions
                dims[0] = rectangles.size() / 6;
                dims[1] = 6;

                // Create the dataspace for a dataset first.
                DataSpace dataspace(2, dims);

                // Create the dataset under group with specified dataspace.      
                DataSet dataset = r_file.createDataSet("rectangles", PredType::NATIVE_DOUBLE, dataspace);

                // Write data in buffer to dataset.
                dataset.write(rectangles.data(), PredType::NATIVE_DOUBLE);
            }
            r_file.close();

            j++;
            if (collision)
                invalid++;
            else 
                valid++;
            labels.push_back(collision);
        }
        std::cout << "done! ..." << std::endl;
    }

    std::cout << "done! ...";

    std::cout << "Saving labels ...";
    { // labels
        // Use H5::hsize_t (similar to int) for dimensions.
        hsize_t dims[1];               // dataset dimensions
        dims[0] = objects * points;

        // Create the dataspace for a dataset first.
        DataSpace dataspace(1, dims);

        // Create the dataset under group with specified dataspace.      
        DataSet dataset = label_file.createDataSet("labels", PredType::NATIVE_DOUBLE, dataspace);

        // Write data in buffer to dataset.
        dataset.write(labels.data(), PredType::NATIVE_DOUBLE);
    }

    { // attribute
        int attr1_data[1] = {int(points)};

        hsize_t attr1_dims[1] = {1};

        // Create the dataspace for an attribute first.
        DataSpace attr1_dataspace(1, attr1_dims);

        // Create the attribute of dataset with specified dataspace.
        Attribute attribute1 = label_file.createAttribute("points", PredType::NATIVE_INT, attr1_dataspace);

        // Write data in buffer to attribute.
        attribute1.write(PredType::NATIVE_INT, attr1_data);
    }

    // Save and exit the file.
    label_file.close();
    std::cout << "done! ..." << std::endl;

    std::cout << "Valid: " << valid << " Invalid: " << invalid << " Valid-Ratio: " << float(valid) / float(valid + invalid) << std::endl;
    return 0;
}

bool argParse(int argc, char** argv, std::string &save_dir, std::size_t &objects, std::size_t &points, std::size_t &expect_obstacles, std::string &shape1, std::string &shape2)
{
    namespace bpo = boost::program_options;

    // Declare the supported options.
    bpo::options_description desc("Allowed options");
    desc.add_options()
        ("help,h", "produce help message")
        ("save_dir", bpo::value<std::string>()->default_value("data"), "The save dir")
        ("objects", bpo::value<std::size_t>()->default_value(1000), "Objects number")
        ("points", bpo::value<std::size_t>()->default_value(1000), "Points number")
        //("bilinear", bpo::value<bool>()->default_value(false), "Bilinear Model")
        ("expect_obstacles", bpo::value<std::size_t>()->default_value(1000), "Specify the expected obstacles number in each scenario")
        ("shape1", bpo::value<std::string>()->default_value("ellipse"), "The first shape type")
        ("shape2", bpo::value<std::string>()->default_value("ellipse"), "The second shape type");
    bpo::variables_map vm;
    bpo::store(bpo::parse_command_line(argc, argv, desc), vm);
    bpo::notify(vm);

    // Check if the help flag has been given:
    if (vm.count("help") != 0u)
    {
        std::cout << desc << std::endl;
        return false;
    }

    save_dir = vm["save_dir"].as<std::string>();
    if (save_dir.back() != '/')
        save_dir += "/";
    objects = vm["objects"].as<std::size_t>();
    points = vm["points"].as<std::size_t>();
    //bilinear = vm["bilinear"].as<bool>();
    expect_obstacles = vm["expect_obstacles"].as<std::size_t>();
    shape1 = vm["shape1"].as<std::string>();
    shape2 = vm["shape2"].as<std::string>();

    return true;
}
