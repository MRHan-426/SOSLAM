//
// Created by ziqihan on 3/2/23.
//
#include <unordered_map>
#include <Eigen/Core>

class SemanticTable {
public:
    SemanticTable() : table_{
        // TODO: modify these scales
        {"can", {0.5, 0.5}},
        {"mouse", {2.0, 1.0}},
        {"keyboard", {10.0, 20.0}},
        {"book", {15.0, 10.0}},
        {"monitor", {0.05, 0.5}},
        {"cup", {0.7, 0.7}},
        {"chair", {0.5, 0.5}},

        {"q0", {5, 5}},
        {"q1", {2, 2}}
        // {"cell phone", {3.0, 4.0}},
        // {"dining table", {3.0, 4.0}}, 
        // {"person", {3.0, 4.0}}, 
        // {"scissors", {3.0, 4.0}}, 
        // {"donut", {3.0, 4.0}}, 
        // {"toothbrush", {3.0, 4.0}}, 
        // {"remote", {3.0, 4.0}}, 
        // {"knife", {3.0, 4.0}}, 
        // {"fork", {3.0, 4.0}}, 
        // {"wine glass", {3.0, 4.0}}, 
        // {"sports ball", {3.0, 4.0}}, 
        // {"handbag", {3.0, 4.0}}, 
        // {"bottle", {3.0, 4.0}}, 
        // {"bed", {3.0, 4.0}}, 
        // {"teddy bear", {3.0, 4.0}}, 
        // {"sink", {3.0, 4.0}},
        // {"hot dog", {3.0, 4.0}}, 
        // {"refrigerator", {3.0, 4.0}}, 
        // {"spoon", {3.0, 4.0}}
        } {}

    // add to table
    void addEntry(std::string label, Eigen::Matrix<double, 2, 1> value) {
        table_[label] = value;
    }

    // get from table
    Eigen::Matrix<double, 2, 1> getEntry(std::string label) {
        return table_[label];
    }

private:
    // table
    std::unordered_map<std::string, Eigen::Matrix<double, 2, 1>> table_;
};
