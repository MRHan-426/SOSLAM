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
        {"keyboard", {8.0, 16.0}},
        {"book", {6.5, 5.5}},
        {"monitor", {0.15, 0.5}},
        {"tape", {1.45, 1.45}},
        {"chair", {0.5, 0.5}},
        {"bowl_smaller",{2.4, 2.4}},
        {"bowl_bigger",{1.8, 1.8}},
        {"cup",{1.2, 1.2}},
        {"laptop",{1.2, 1.4}},
        {"white_mouse",{2.0, 1.0}},
        {"black_mouse",{2.0, 1.0}},
        {"game_remote",{3.0, 4.0}},
        {"white_keyboard",{8.0, 16.0}},
        {"laptop_2",{9.0, 12.0}},
        {"bottle",{0.3, 0.3}},
        {"white_book", {6.5, 5.5}},
        {"cube_book", {6.5, 5.5}},
        {"blue_book", {6.5, 5.5}},
        {"ass_book", {6.5, 5.5}},
//        {"lap_top",{1.2, 1.4}},

            {"q0",       {5,    5}},
            {"q1",       {2,    2}}
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
