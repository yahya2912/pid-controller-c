#include <gtk/gtk.h>
#include <math.h>
#include <string.h>
#include "pid.h"

#define SIM_STEPS   300
#define DT          0.05f

static double g_output[SIM_STEPS];
static double g_setpoint_line[SIM_STEPS];
static int    g_sim_done = 0;

static GtkWidget *spin_kp, *spin_ki, *spin_kd, *spin_sp;
static GtkWidget *drawing_area;

static gboolean on_draw(GtkWidget *widget, cairo_t *cr, gpointer data)
{
    (void)data;
    int w = gtk_widget_get_allocated_width(widget);
    int h = gtk_widget_get_allocated_height(widget);

    cairo_set_source_rgb(cr, 0.12, 0.12, 0.15);
    cairo_paint(cr);

    if (!g_sim_done) return FALSE;

    double sp = g_setpoint_line[0];
    double ymin = sp, ymax = sp;
    for (int i = 0; i < SIM_STEPS; i++) {
        if (g_output[i] < ymin) ymin = g_output[i];
        if (g_output[i] > ymax) ymax = g_output[i];
    }
    double margin = (ymax - ymin) * 0.15 + 0.5;
    ymin -= margin; ymax += margin;

    double xscale = (double)(w - 40) / (SIM_STEPS - 1);
    double yscale = (h - 40) / (ymax - ymin);

    #define TX(i)  (20 + (i) * xscale)
    #define TY(v)  (h - 20 - ((v) - ymin) * yscale)

    cairo_set_source_rgb(cr, 0.3, 0.7, 0.3);
    cairo_set_line_width(cr, 1.5);
    cairo_move_to(cr, TX(0), TY(sp));
    cairo_line_to(cr, TX(SIM_STEPS-1), TY(sp));
    cairo_stroke(cr);

    cairo_set_source_rgb(cr, 0.2, 0.6, 1.0);
    cairo_set_line_width(cr, 2.0);
    cairo_move_to(cr, TX(0), TY(g_output[0]));
    for (int i = 1; i < SIM_STEPS; i++)
        cairo_line_to(cr, TX(i), TY(g_output[i]));
    cairo_stroke(cr);

    cairo_set_font_size(cr, 11);
    cairo_set_source_rgb(cr, 0.3, 0.7, 0.3);
    cairo_move_to(cr, 25, 18); cairo_show_text(cr, "-- Setpoint");
    cairo_set_source_rgb(cr, 0.2, 0.6, 1.0);
    cairo_move_to(cr, 120, 18); cairo_show_text(cr, "-- PV (output)");

    return FALSE;
}

static void on_run_clicked(GtkButton *btn, gpointer data)
{
    (void)btn; (void)data;

    double kp = gtk_spin_button_get_value(GTK_SPIN_BUTTON(spin_kp));
    double ki = gtk_spin_button_get_value(GTK_SPIN_BUTTON(spin_ki));
    double kd = gtk_spin_button_get_value(GTK_SPIN_BUTTON(spin_kd));
    double sp = gtk_spin_button_get_value(GTK_SPIN_BUTTON(spin_sp));

    PID_params pid;
    PID_init(&pid, (float)kp, (float)ki, (float)kd, -100.0f, 100.0f);

    float plant_value = 0.0f;
    float tau = 1.0f;

    for (int i = 0; i < SIM_STEPS; i++) {
        float u = PID_update(&pid, (float)sp, plant_value, DT);
        if (u >  100.0f) u =  100.0f;
        if (u < -100.0f) u = -100.0f;
        plant_value += ((u - plant_value) / tau) * DT;
        g_output[i]        = plant_value;
        g_setpoint_line[i] = sp;
    }

    g_sim_done = 1;
    gtk_widget_queue_draw(drawing_area);
}

static GtkWidget *make_spin(const char *label, double min, double max,
                             double step, double val, GtkWidget **out)
{
    GtkWidget *box  = gtk_box_new(GTK_ORIENTATION_HORIZONTAL, 6);
    GtkWidget *lbl  = gtk_label_new(label);
    GtkWidget *spin = gtk_spin_button_new_with_range(min, max, step);
    gtk_spin_button_set_value(GTK_SPIN_BUTTON(spin), val);
    gtk_spin_button_set_digits(GTK_SPIN_BUTTON(spin), 3);
    gtk_label_set_width_chars(GTK_LABEL(lbl), 10);
    gtk_box_pack_start(GTK_BOX(box), lbl,  FALSE, FALSE, 0);
    gtk_box_pack_start(GTK_BOX(box), spin, TRUE,  TRUE,  0);
    if (out) *out = spin;
    return box;
}

int main(int argc, char *argv[])
{
    gtk_init(&argc, &argv);

    GtkWidget *window = gtk_window_new(GTK_WINDOW_TOPLEVEL);
    gtk_window_set_title(GTK_WINDOW(window), "PID Controller");
    gtk_window_set_default_size(GTK_WINDOW(window), 700, 520);
    g_signal_connect(window, "destroy", G_CALLBACK(gtk_main_quit), NULL);

    GtkWidget *vbox = gtk_box_new(GTK_ORIENTATION_VERTICAL, 10);
    gtk_container_set_border_width(GTK_CONTAINER(vbox), 12);
    gtk_container_add(GTK_CONTAINER(window), vbox);

    GtkWidget *frame     = gtk_frame_new("PID Parameters");
    GtkWidget *param_box = gtk_box_new(GTK_ORIENTATION_VERTICAL, 6);
    gtk_container_set_border_width(GTK_CONTAINER(param_box), 8);
    gtk_container_add(GTK_CONTAINER(frame), param_box);

    gtk_box_pack_start(GTK_BOX(param_box),
        make_spin("Kp",       0.0,  200.0, 0.1,   2.0, &spin_kp), FALSE, FALSE, 0);
    gtk_box_pack_start(GTK_BOX(param_box),
        make_spin("Ki",       0.0,  200.0, 0.1,   0.5, &spin_ki), FALSE, FALSE, 0);
    gtk_box_pack_start(GTK_BOX(param_box),
        make_spin("Kd",       0.0,  200.0, 0.1,   0.1, &spin_kd), FALSE, FALSE, 0);
    gtk_box_pack_start(GTK_BOX(param_box),
        make_spin("Setpoint", -500.0, 500.0, 1.0, 100.0, &spin_sp), FALSE, FALSE, 0);

    gtk_box_pack_start(GTK_BOX(vbox), frame, FALSE, FALSE, 0);

    GtkWidget *btn = gtk_button_new_with_label("Run Simulation");
    g_signal_connect(btn, "clicked", G_CALLBACK(on_run_clicked), NULL);
    gtk_box_pack_start(GTK_BOX(vbox), btn, FALSE, FALSE, 0);

    GtkWidget *plot_frame = gtk_frame_new("Response Curve");
    drawing_area = gtk_drawing_area_new();
    gtk_widget_set_size_request(drawing_area, -1, 320);
    g_signal_connect(drawing_area, "draw", G_CALLBACK(on_draw), NULL);
    gtk_container_add(GTK_CONTAINER(plot_frame), drawing_area);
    gtk_box_pack_start(GTK_BOX(vbox), plot_frame, TRUE, TRUE, 0);

    gtk_widget_show_all(window);
    gtk_main();
    return 0;
}
