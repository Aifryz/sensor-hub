#include "table_screen.hpp"
#include <drivers/peripheral/encoder.hpp>

#include <sensors/sensors_db.hpp>

#include <cstdio>
#include <cstring>
#include <cstdlib>
#include <src/widgets/label/lv_label.h>


lv_obj_t* table;

void lv_example_get_started_1(void)
{
    //Change the active screen's background color
    //lv_scr_act();
    lv_obj_set_style_bg_color(lv_scr_act(), lv_color_hex(0x003a57), LV_PART_MAIN);

    //Create a white label, set its text and align it to the center
    lv_obj_t * label = lv_label_create(lv_scr_act());
    lv_label_set_text(label, "Hello world");
    lv_obj_set_style_text_color(lv_scr_act(), lv_color_hex(0xffffff), LV_PART_MAIN);
    lv_obj_align(label, LV_ALIGN_CENTER, 0, 0);
}

void lv_example_create_table(void)
{
    //Create a table
    lv_obj_t * table = lv_table_create(lv_scr_act());
    lv_obj_set_size(table, 320, 240);
    lv_obj_center(table);

    //Set the number of rows and columns
    lv_table_set_row_cnt(table, 3);
    lv_table_set_col_cnt(table, 3);

    //Fill the first row
    lv_table_set_cell_value(table, 0, 0, "R1C1");
    lv_table_set_cell_value(table, 0, 1, "R1C2");
    lv_table_set_cell_value(table, 0, 2, "R1C3");

    //Fill the second row
    lv_table_set_cell_value(table, 1, 0, "R2C1");
    lv_table_set_cell_value(table, 1, 1, "R2C2");
    lv_table_set_cell_value(table, 1, 2, "R2C3");

    //Fill the third row
    lv_table_set_cell_value(table, 2, 0, "R3C1");
    lv_table_set_cell_value(table, 2, 1, "R3C2");
    lv_table_set_cell_value(table, 2, 2, "R3C3");
}

lv_obj_t *time_label;

void create_table()
{
    table = lv_table_create(lv_scr_act());
    lv_obj_set_pos(table, 0, 25);
    lv_table_set_row_cnt(table, 5);
    lv_table_set_col_cnt(table, 6);
    lv_table_set_col_width(table, 0, 60);
    lv_table_set_col_width(table, 1, 60);
    lv_table_set_col_width(table, 2, 60);
    lv_table_set_col_width(table, 3, 40);
    lv_table_set_col_width(table, 4, 60);
    lv_table_set_col_width(table, 5, 40);

    lv_obj_t *cont = lv_obj_create(lv_scr_act());
    lv_obj_set_width(cont, 320);
    lv_obj_set_height(cont, 25);

    static lv_style_t style_cont;
    lv_style_init(&style_cont);

    //lv_style_set_bg_color(&style_cont, white);
    lv_style_set_border_width(&style_cont, 1);
    //lv_style_set_border_color(&style_cont, gray);
    lv_style_set_pad_all(&style_cont, 2);
    lv_obj_add_style(cont, &style_cont, 0);

    static lv_style_t style_table_pad;
    lv_style_init(&style_table_pad);
    lv_style_set_pad_left(&style_table_pad, 2);
    lv_style_set_pad_right(&style_table_pad, 2);
    lv_obj_add_style(table, &style_table_pad, LV_PART_ITEMS);

    time_label = lv_label_create(cont);

    lv_label_set_text(time_label, "Time:");

    static const char *const locations[5] = {"Okno", "Dwor", "Balkon", "Pokoj",
                                             "Strych"};

    for (int i = 0; i < 5; i++)
    {
        lv_table_set_cell_value(table, i, 0, locations[i]);
    }

}

void format100(char* buf, int16_t val, const char* tag)
{
    uint32_t aval = abs(val);
    if (val >= 0)
    {
        sprintf(buf, "%d.%d%s", aval / 100, aval % 100, tag);
    }
    else
    {
        sprintf(buf, "-%d.%d%s", aval / 100, aval % 100, tag);
    }
}

void format1000(char *buf, int16_t val, const char* tag)
{
    uint32_t aval = abs(val);
    if (val >= 0)
    {
        sprintf(buf, "%d.%d%s", aval / 1000, aval % 1000, tag);
    }
    else
    {
        sprintf(buf, "-%d.%d%s", aval / 1000, aval % 1000, tag);
    }
}

static void updateTimeLabel()
{
    char buf[32];
    uint32_t tick = enc.get_ticks();
    sprintf(buf, "Time: %d.%03d s", tick / 1000, tick % 1000);
    lv_label_set_text(time_label, buf);
}

void updateTableData()
{

   updateTimeLabel();

    char str[32];
    for (int col = 0; col < 5; col++)
    {
        int node = col + 1;

        auto& sensor_db = GetSensorDB();

        auto temperature = sensor_db.GetLastMeasurement(node, MeasurementType::Temperature);
        auto humidity = sensor_db.GetLastMeasurement(node, MeasurementType::Humidity);
        auto light = sensor_db.GetLastMeasurement(node, MeasurementType::Light);
        auto battery = sensor_db.GetLastMeasurement(node, MeasurementType::Battery);


        if(temperature.has_value())
        {
            format100(str, temperature.value(), "C");
        }
        else {
            sprintf(str, "---");
        }
        lv_table_set_cell_value(table, col, 1, str);

        if(humidity.has_value())
        {
            format100(str, humidity.value(), "%");
        }
        else {
            sprintf(str, "---");
        }
        lv_table_set_cell_value(table, col, 2, str);
        
        if (light.has_value())
        {
           sprintf(str,"%d", light.value());
        }
        else
        {
            sprintf(str, "---");
        }
        lv_table_set_cell_value(table, col, 3, str);

        if (battery.has_value())
        {
            format1000(str, battery.value(), "V");
        }
        else
        {
            sprintf(str, "---");
        }
        lv_table_set_cell_value(table, col, 4, str);

    }
}