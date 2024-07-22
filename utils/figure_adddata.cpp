#include <mujoco/mujoco.h>

#include <iostream>
#include <vector>
#include <string>
#include <stdexcept>


void add_data_to_line(mjrRect viewport_fig, mjrContext* context, mjvFigure fig,  int line_idx, double line_data, int timestep) {
    // Increment point count, keeping it within maximum
    int pnt = std::min(mjMAXLINE, fig.linepnt[line_idx] + 1);

    // Shift data
    for (int i = pnt - 1; i > 0; --i) {
        fig.linedata[line_idx][2 * i + 1] = fig.linedata[line_idx][2 * i - 1];
    }

    // Assign new data
    fig.linepnt[line_idx] = pnt;
    fig.linedata[line_idx][0] = timestep;
    fig.linedata[line_idx][1] = line_data;

    mjr_figure(viewport_fig, &fig, context);
}


// // before while loop
// // ---------------------------------------------------------
//     // Add a graph detailing A1's position
//     mjvFigure fig_a1pos;
//     mjv_defaultFigure(&fig_a1pos);

//     // flags
//     fig_a1pos.flg_legend = true; // show legend
//     fig_a1pos.flg_extend = true; // automatically extend axis ranges to fit data

//     // figure title
//     strcpy(fig_a1pos.title, "A1 Global Position");
//     // figure x-label
//     // strcpy(fig_a1pos.xlabel, "Timestep");
//     // figure x-value range
//     // fig_a1pos.range[0][0] = -2; // x
//     // fig_a1pos.range[0][1] = 10; // x
//     // figure y-value range
//     // fig_a1pos.range[1][0] = -2; // y
//     // fig_a1pos.range[1][1] = 12; // y
//     fig_a1pos.linepnt[0] = mjMAXLINEPNT; // Points in the line 0
//     fig_a1pos.linepnt[1] = mjMAXLINEPNT; // Points in the line 1
//     // figure background color
//     // fig_a1pos.figurergba[0] = 0.1f;
//     // figure line color
//     // fig_a1pos.linergb[0][0] = 0.6f;
//     // figure grid size
//     // fig_a1pos.gridsize[0] = (fig_a1pos.range[0][1] - fig_a1pos.range[0][0]) + 1; // N-1
//     // fig_a1pos.gridsize[1] = (fig_a1pos.range[1][1] - fig_a1pos.range[1][0]) + 1; // N-1
//     // tick labels visible/invisible
//     // fig_a1pos.flg_ticklabel[0] = false;
//     // fig_a1pos.flg_ticklabel[1] = false;

//     mjrRect viewport_fig = {0, 0, 300,300};
//     mjr_figure(viewport_fig, &fig_a1pos, &con);
    
//     strcpy(fig_a1pos.linename[0], "robot_X-pos");
//     strcpy(fig_a1pos.linename[1], "robot_Y-pos");
//     fig_a1pos.linedata[0][1] = modelstate.bodyWorldPos.x; // x
//     fig_a1pos.linedata[1][1] = modelstate.bodyWorldPos.y; // y
//     // ---------------------------------------------------------

// // in while loop
// // add plot to screen
//         // mjr_figure(viewport_fig, &fig_a1pos, &con);
//         double linedata = modelstate.bodyWorldPos.x;
//         add_data_to_line(viewport_fig, &con, fig_a1pos, 0, linedata, timestep);
//         linedata = modelstate.bodyWorldPos.y;
//         // add_data_to_line(fig_a1pos, 1, linedata);
//         add_data_to_line(viewport_fig, &con, fig_a1pos, 1, linedata, timestep);
//         // mjr_figure(viewport_fig, &fig_a1pos, &con);

//         // double line_data[] = {modelstate.bodyWorldPos.x, modelstate.bodyWorldPos.y};
//         // fig_a1pos.linedata[0][0] = modelstate.bodyWorldPos.x; // x
//         // fig_a1pos.linedata[0][1] = modelstate.bodyWorldPos.y; // y
//         // Increment point count, keeping it within maximum
//         // int line_idx = 0;
//         // int pnt = std::min(mjMAXLINE, fig_a1pos.linepnt[line_idx] + 1);
        
//         // // Shift data
//         // for (int i = pnt - 1; i > 0; --i) {
//         //     fig_a1pos.linedata[0][2 * i + 1] = fig_a1pos.linedata[0][2 * i - 1];
//         // }

//         // // Assign new data
//         // fig_a1pos.linepnt[line_idx] = pnt;
//         // fig_a1pos.linedata[line_idx][1] = line_data[0];
        
//         // pnt = std::min(mjMAXLINE, fig_a1pos.linepnt[line_idx+1] + 1);
        
//         // // Shift data
//         // for (int i = pnt - 1; i > 0; --i) {
//         //     fig_a1pos.linedata[1][2 * i + 1] = fig_a1pos.linedata[1][2 * i - 1];
//         // }
        
//         // fig_a1pos.linepnt[line_idx+1] = pnt;
//         // fig_a1pos.linedata[line_idx+1][1] = line_data[1];