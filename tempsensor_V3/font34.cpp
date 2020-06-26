//file: font34.cpp
//
// Bitmap font C source generated by bmfont2c.py
// Converted from Arial Black by bmfont and bmtfontgen.py
// 

#include <Arduino.h>
#include <stdint.h>
#include <avr/pgmspace.h>
#include "display.h"

// A 34-raster high fixed width font with only digits and the degree symbol

const byte font34_data [] PROGMEM = {

    // No glyph for ASCII: 47, using substitute:
    // **** NEED TO TWEAK THIS TO BE THE DEGREE SYMBOL, IF WE USE IT ****
    // ASCII: 37, char width: 45
    0x01, 0xfc, 0x00, 0x03,  // -------OOOOOOO----------------OO
    0x07, 0xff, 0x00, 0x03,  // -----OOOOOOOOOOO--------------OO
    0x0f, 0xff, 0x80, 0x07,  // ----OOOOOOOOOOOOO------------OOO
    0x1f, 0xff, 0xc0, 0x07,  // ---OOOOOOOOOOOOOOO-----------OOO
    0x1f, 0x8f, 0xc0, 0x0f,  // ---OOOOOO---OOOOOO----------OOOO
    0x3f, 0x07, 0xe0, 0x1f,  // --OOOOOO-----OOOOOO--------OOOOO
    0x3e, 0x03, 0xe0, 0x1e,  // --OOOOO-------OOOOO--------OOOO-
    0x3e, 0x03, 0xe0, 0x3e,  // --OOOOO-------OOOOO-------OOOOO-
    0x3e, 0x03, 0xe0, 0x3c,  // --OOOOO-------OOOOO-------OOOO--
    0x3e, 0x03, 0xe0, 0x7c,  // --OOOOO-------OOOOO------OOOOO--
    0x3e, 0x03, 0xe0, 0x78,  // --OOOOO-------OOOOO------OOOO---
    0x3f, 0x07, 0xe0, 0xf0,  // --OOOOOO-----OOOOOO-----OOOO----
    0x1f, 0x8f, 0xc1, 0xf0,  // ---OOOOOO---OOOOOO-----OOOOO----
    0x1f, 0xff, 0xc1, 0xe0,  // ---OOOOOOOOOOOOOOO-----OOOO-----
    0x0f, 0xff, 0x83, 0xe0,  // ----OOOOOOOOOOOOO-----OOOOO-----
    0x07, 0xff, 0x03, 0xc0,  // -----OOOOOOOOOOO------OOOO------
    0x01, 0xfc, 0x07, 0xc0,  // -------OOOOOOO-------OOOOO------
    0x00, 0x00, 0x0f, 0x81,  // --------------------OOOOO------O
    0x00, 0x00, 0x0f, 0x07,  // --------------------OOOO-----OOO
    0x00, 0x00, 0x1f, 0x0f,  // -------------------OOOOO----OOOO
    0x00, 0x00, 0x1e, 0x1f,  // -------------------OOOO----OOOOO
    0x00, 0x00, 0x3e, 0x1f,  // ------------------OOOOO----OOOOO
    0x00, 0x00, 0x3c, 0x3f,  // ------------------OOOO----OOOOOO
    0x00, 0x00, 0x78, 0x3e,  // -----------------OOOO-----OOOOO-
    0x00, 0x00, 0xf8, 0x3e,  // ----------------OOOOO-----OOOOO-
    0x00, 0x00, 0xf0, 0x3e,  // ----------------OOOO------OOOOO-
    0x00, 0x01, 0xf0, 0x3e,  // ---------------OOOOO------OOOOO-
    0x00, 0x01, 0xe0, 0x3e,  // ---------------OOOO-------OOOOO-
    0x00, 0x03, 0xe0, 0x3f,  // --------------OOOOO-------OOOOOO
    0x00, 0x03, 0xc0, 0x1f,  // --------------OOOO---------OOOOO
    0x00, 0x07, 0x80, 0x1f,  // -------------OOOO----------OOOOO
    0x00, 0x0f, 0x80, 0x0f,  // ------------OOOOO-----------OOOO
    0x00, 0x0f, 0x00, 0x07,  // ------------OOOO-------------OOO
    0x00, 0x1f, 0x00, 0x01,  // -----------OOOOO---------------O

    // ASCII: 48, char width: 30
    0x00, 0x1f, 0xf0, 0x00,  // -----------OOOOOOOOO----------..
    0x00, 0xff, 0xfc, 0x00,  // --------OOOOOOOOOOOOOO--------..
    0x01, 0xff, 0xfe, 0x00,  // -------OOOOOOOOOOOOOOOO-------..
    0x07, 0xff, 0xff, 0x00,  // -----OOOOOOOOOOOOOOOOOOO------..
    0x07, 0xff, 0xff, 0x80,  // -----OOOOOOOOOOOOOOOOOOOO-----..
    0x0f, 0xff, 0xff, 0xc0,  // ----OOOOOOOOOOOOOOOOOOOOOO----..
    0x0f, 0xf8, 0x7f, 0xc0,  // ----OOOOOOOOO----OOOOOOOOO----..
    0x1f, 0xf0, 0x3f, 0xe0,  // ---OOOOOOOOO------OOOOOOOOO---..
    0x1f, 0xf0, 0x3f, 0xe0,  // ---OOOOOOOOO------OOOOOOOOO---..
    0x1f, 0xf0, 0x3f, 0xe0,  // ---OOOOOOOOO------OOOOOOOOO---..
    0x3f, 0xe0, 0x1f, 0xe0,  // --OOOOOOOOO--------OOOOOOOO---..
    0x3f, 0xe0, 0x1f, 0xf0,  // --OOOOOOOOO--------OOOOOOOOO--..
    0x3f, 0xe0, 0x1f, 0xf0,  // --OOOOOOOOO--------OOOOOOOOO--..
    0x3f, 0xe0, 0x1f, 0xf0,  // --OOOOOOOOO--------OOOOOOOOO--..
    0x3f, 0xe0, 0x1f, 0xf0,  // --OOOOOOOOO--------OOOOOOOOO--..
    0x3f, 0xe0, 0x1f, 0xf0,  // --OOOOOOOOO--------OOOOOOOOO--..
    0x3f, 0xe0, 0x1f, 0xf0,  // --OOOOOOOOO--------OOOOOOOOO--..
    0x3f, 0xe0, 0x1f, 0xf0,  // --OOOOOOOOO--------OOOOOOOOO--..
    0x3f, 0xe0, 0x1f, 0xf0,  // --OOOOOOOOO--------OOOOOOOOO--..
    0x3f, 0xe0, 0x1f, 0xf0,  // --OOOOOOOOO--------OOOOOOOOO--..
    0x3f, 0xe0, 0x1f, 0xf0,  // --OOOOOOOOO--------OOOOOOOOO--..
    0x3f, 0xe0, 0x1f, 0xf0,  // --OOOOOOOOO--------OOOOOOOOO--..
    0x3f, 0xe0, 0x1f, 0xf0,  // --OOOOOOOOO--------OOOOOOOOO--..
    0x1f, 0xe0, 0x1f, 0xe0,  // ---OOOOOOOO--------OOOOOOOO---..
    0x1f, 0xf0, 0x3f, 0xe0,  // ---OOOOOOOOO------OOOOOOOOO---..
    0x1f, 0xf0, 0x3f, 0xe0,  // ---OOOOOOOOO------OOOOOOOOO---..
    0x1f, 0xf0, 0x3f, 0xe0,  // ---OOOOOOOOO------OOOOOOOOO---..
    0x0f, 0xf8, 0x7f, 0xc0,  // ----OOOOOOOOO----OOOOOOOOO----..
    0x0f, 0xff, 0xff, 0xc0,  // ----OOOOOOOOOOOOOOOOOOOOOO----..
    0x07, 0xff, 0xff, 0x80,  // -----OOOOOOOOOOOOOOOOOOOO-----..
    0x03, 0xff, 0xff, 0x80,  // ------OOOOOOOOOOOOOOOOOOO-----..
    0x01, 0xff, 0xff, 0x00,  // -------OOOOOOOOOOOOOOOOO------..
    0x00, 0xff, 0xfc, 0x00,  // --------OOOOOOOOOOOOOO--------..
    0x00, 0x1f, 0xe0, 0x00,  // -----------OOOOOOOO-----------..

    // ASCII: 49, char width: 30
    0x00, 0x03, 0xfc, 0x00,  // --------------OOOOOOOO--------..
    0x00, 0x03, 0xfc, 0x00,  // --------------OOOOOOOO--------..
    0x00, 0x07, 0xfc, 0x00,  // -------------OOOOOOOOO--------..
    0x00, 0x07, 0xfc, 0x00,  // -------------OOOOOOOOO--------..
    0x00, 0x0f, 0xfc, 0x00,  // ------------OOOOOOOOOO--------..
    0x00, 0x1f, 0xfc, 0x00,  // -----------OOOOOOOOOOO--------..
    0x00, 0x7f, 0xfc, 0x00,  // ---------OOOOOOOOOOOOO--------..
    0x00, 0xff, 0xfc, 0x00,  // --------OOOOOOOOOOOOOO--------..
    0x07, 0xff, 0xfc, 0x00,  // -----OOOOOOOOOOOOOOOOO--------..
    0x0f, 0xff, 0xfc, 0x00,  // ----OOOOOOOOOOOOOOOOOO--------..
    0x0f, 0xff, 0xfc, 0x00,  // ----OOOOOOOOOOOOOOOOOO--------..
    0x0f, 0xff, 0xfc, 0x00,  // ----OOOOOOOOOOOOOOOOOO--------..
    0x0f, 0xf7, 0xfc, 0x00,  // ----OOOOOOOO-OOOOOOOOO--------..
    0x0f, 0xc7, 0xfc, 0x00,  // ----OOOOOO---OOOOOOOOO--------..
    0x0f, 0x07, 0xfc, 0x00,  // ----OOOO-----OOOOOOOOO--------..
    0x0c, 0x07, 0xfc, 0x00,  // ----OO-------OOOOOOOOO--------..
    0x00, 0x07, 0xfc, 0x00,  // -------------OOOOOOOOO--------..
    0x00, 0x07, 0xfc, 0x00,  // -------------OOOOOOOOO--------..
    0x00, 0x07, 0xfc, 0x00,  // -------------OOOOOOOOO--------..
    0x00, 0x07, 0xfc, 0x00,  // -------------OOOOOOOOO--------..
    0x00, 0x07, 0xfc, 0x00,  // -------------OOOOOOOOO--------..
    0x00, 0x07, 0xfc, 0x00,  // -------------OOOOOOOOO--------..
    0x00, 0x07, 0xfc, 0x00,  // -------------OOOOOOOOO--------..
    0x00, 0x07, 0xfc, 0x00,  // -------------OOOOOOOOO--------..
    0x00, 0x07, 0xfc, 0x00,  // -------------OOOOOOOOO--------..
    0x00, 0x07, 0xfc, 0x00,  // -------------OOOOOOOOO--------..
    0x00, 0x07, 0xfc, 0x00,  // -------------OOOOOOOOO--------..
    0x00, 0x07, 0xfc, 0x00,  // -------------OOOOOOOOO--------..
    0x00, 0x07, 0xfc, 0x00,  // -------------OOOOOOOOO--------..
    0x00, 0x07, 0xfc, 0x00,  // -------------OOOOOOOOO--------..
    0x00, 0x07, 0xfc, 0x00,  // -------------OOOOOOOOO--------..
    0x00, 0x07, 0xfc, 0x00,  // -------------OOOOOOOOO--------..
    0x00, 0x07, 0xfc, 0x00,  // -------------OOOOOOOOO--------..
    0x00, 0x00, 0x00, 0x00,  // ------------------------------..

    // ASCII: 50, char width: 30
    0x00, 0x1f, 0xf8, 0x00,  // -----------OOOOOOOOOO---------..
    0x00, 0xff, 0xfe, 0x00,  // --------OOOOOOOOOOOOOOO-------..
    0x03, 0xff, 0xff, 0x80,  // ------OOOOOOOOOOOOOOOOOOO-----..
    0x07, 0xff, 0xff, 0xc0,  // -----OOOOOOOOOOOOOOOOOOOOO----..
    0x07, 0xff, 0xff, 0xe0,  // -----OOOOOOOOOOOOOOOOOOOOOO---..
    0x0f, 0xff, 0xff, 0xe0,  // ----OOOOOOOOOOOOOOOOOOOOOOO---..
    0x0f, 0xfc, 0x7f, 0xf0,  // ----OOOOOOOOOO---OOOOOOOOOOO--..
    0x0f, 0xf8, 0x3f, 0xf0,  // ----OOOOOOOOO-----OOOOOOOOOO--..
    0x1f, 0xf8, 0x1f, 0xf0,  // ---OOOOOOOOOO------OOOOOOOOO--..
    0x1f, 0xf0, 0x1f, 0xf0,  // ---OOOOOOOOO-------OOOOOOOOO--..
    0x03, 0xf0, 0x1f, 0xf0,  // ------OOOOOO-------OOOOOOOOO--..
    0x00, 0x00, 0x1f, 0xf0,  // -------------------OOOOOOOOO--..
    0x00, 0x00, 0x3f, 0xf0,  // ------------------OOOOOOOOOO--..
    0x00, 0x00, 0x7f, 0xe0,  // -----------------OOOOOOOOOO---..
    0x00, 0x00, 0x7f, 0xe0,  // -----------------OOOOOOOOOO---..
    0x00, 0x01, 0xff, 0xc0,  // ---------------OOOOOOOOOOO----..
    0x00, 0x03, 0xff, 0x80,  // --------------OOOOOOOOOOO-----..
    0x00, 0x07, 0xff, 0x80,  // -------------OOOOOOOOOOOO-----..
    0x00, 0x0f, 0xff, 0x00,  // ------------OOOOOOOOOOOO------..
    0x00, 0x1f, 0xfc, 0x00,  // -----------OOOOOOOOOOO--------..
    0x00, 0x7f, 0xf8, 0x00,  // ---------OOOOOOOOOOOO---------..
    0x00, 0xff, 0xf0, 0x00,  // --------OOOOOOOOOOOO----------..
    0x01, 0xff, 0xe0, 0x00,  // -------OOOOOOOOOOOO-----------..
    0x03, 0xff, 0x80, 0x00,  // ------OOOOOOOOOOO-------------..
    0x03, 0xff, 0x00, 0x00,  // ------OOOOOOOOOO--------------..
    0x07, 0xfe, 0x00, 0x00,  // -----OOOOOOOOOO---------------..
    0x0f, 0xff, 0xff, 0xf0,  // ----OOOOOOOOOOOOOOOOOOOOOOOO--..
    0x0f, 0xff, 0xff, 0xf0,  // ----OOOOOOOOOOOOOOOOOOOOOOOO--..
    0x1f, 0xff, 0xff, 0xf0,  // ---OOOOOOOOOOOOOOOOOOOOOOOOO--..
    0x1f, 0xff, 0xff, 0xf0,  // ---OOOOOOOOOOOOOOOOOOOOOOOOO--..
    0x3f, 0xff, 0xff, 0xf0,  // --OOOOOOOOOOOOOOOOOOOOOOOOOO--..
    0x3f, 0xff, 0xff, 0xf0,  // --OOOOOOOOOOOOOOOOOOOOOOOOOO--..
    0x3f, 0xff, 0xff, 0xf0,  // --OOOOOOOOOOOOOOOOOOOOOOOOOO--..
    0x00, 0x00, 0x00, 0x00,  // ------------------------------..

    // ASCII: 51, char width: 30
    0x00, 0x3f, 0xe0, 0x00,  // ----------OOOOOOOOO-----------..
    0x01, 0xff, 0xfc, 0x00,  // -------OOOOOOOOOOOOOOO--------..
    0x03, 0xff, 0xfe, 0x00,  // ------OOOOOOOOOOOOOOOOO-------..
    0x07, 0xff, 0xff, 0x00,  // -----OOOOOOOOOOOOOOOOOOO------..
    0x0f, 0xff, 0xff, 0x80,  // ----OOOOOOOOOOOOOOOOOOOOO-----..
    0x1f, 0xff, 0xff, 0x80,  // ---OOOOOOOOOOOOOOOOOOOOOO-----..
    0x1f, 0xf8, 0xff, 0xc0,  // ---OOOOOOOOOO---OOOOOOOOOO----..
    0x3f, 0xf0, 0x7f, 0xc0,  // --OOOOOOOOOO-----OOOOOOOOO----..
    0x3f, 0xe0, 0x7f, 0xc0,  // --OOOOOOOOO------OOOOOOOOO----..
    0x00, 0xe0, 0x7f, 0xc0,  // --------OOO------OOOOOOOOO----..
    0x00, 0x00, 0x7f, 0xc0,  // -----------------OOOOOOOOO----..
    0x00, 0x00, 0xff, 0x80,  // ----------------OOOOOOOOO-----..
    0x00, 0x01, 0xff, 0x80,  // ---------------OOOOOOOOOO-----..
    0x00, 0x0f, 0xff, 0x00,  // ------------OOOOOOOOOOOO------..
    0x00, 0x0f, 0xfe, 0x00,  // ------------OOOOOOOOOOO-------..
    0x00, 0x0f, 0xfc, 0x00,  // ------------OOOOOOOOOO--------..
    0x00, 0x0f, 0xff, 0x00,  // ------------OOOOOOOOOOOO------..
    0x00, 0x0f, 0xff, 0xc0,  // ------------OOOOOOOOOOOOOO----..
    0x00, 0x0f, 0xff, 0xe0,  // ------------OOOOOOOOOOOOOOO---..
    0x00, 0x00, 0x7f, 0xe0,  // -----------------OOOOOOOOOO---..
    0x00, 0x00, 0x3f, 0xf0,  // ------------------OOOOOOOOOO--..
    0x00, 0x00, 0x1f, 0xf0,  // -------------------OOOOOOOOO--..
    0x00, 0x00, 0x1f, 0xf0,  // -------------------OOOOOOOOO--..
    0x00, 0x00, 0x1f, 0xf0,  // -------------------OOOOOOOOO--..
    0x03, 0xe0, 0x1f, 0xf0,  // ------OOOOO--------OOOOOOOOO--..
    0x3f, 0xe0, 0x1f, 0xf0,  // --OOOOOOOOO--------OOOOOOOOO--..
    0x1f, 0xf0, 0x3f, 0xe0,  // ---OOOOOOOOO------OOOOOOOOO---..
    0x1f, 0xf8, 0x7f, 0xe0,  // ---OOOOOOOOOO----OOOOOOOOOO---..
    0x1f, 0xff, 0xff, 0xe0,  // ---OOOOOOOOOOOOOOOOOOOOOOOO---..
    0x0f, 0xff, 0xff, 0xc0,  // ----OOOOOOOOOOOOOOOOOOOOOO----..
    0x07, 0xff, 0xff, 0x80,  // -----OOOOOOOOOOOOOOOOOOOO-----..
    0x03, 0xff, 0xff, 0x00,  // ------OOOOOOOOOOOOOOOOOO------..
    0x00, 0xff, 0xfe, 0x00,  // --------OOOOOOOOOOOOOOO-------..
    0x00, 0x3f, 0xf0, 0x00,  // ----------OOOOOOOOOO----------..

    // ASCII: 52, char width: 30
    0x00, 0x00, 0x7f, 0xc0,  // -----------------OOOOOOOOO----..
    0x00, 0x00, 0xff, 0xc0,  // ----------------OOOOOOOOOO----..
    0x00, 0x01, 0xff, 0xc0,  // ---------------OOOOOOOOOOO----..
    0x00, 0x03, 0xff, 0xc0,  // --------------OOOOOOOOOOOO----..
    0x00, 0x07, 0xff, 0xc0,  // -------------OOOOOOOOOOOOO----..
    0x00, 0x07, 0xff, 0xc0,  // -------------OOOOOOOOOOOOO----..
    0x00, 0x0f, 0xff, 0xc0,  // ------------OOOOOOOOOOOOOO----..
    0x00, 0x1f, 0xff, 0xc0,  // -----------OOOOOOOOOOOOOOO----..
    0x00, 0x3f, 0xff, 0xc0,  // ----------OOOOOOOOOOOOOOOO----..
    0x00, 0x7f, 0xff, 0xc0,  // ---------OOOOOOOOOOOOOOOOO----..
    0x00, 0xff, 0x7f, 0xc0,  // --------OOOOOOOO-OOOOOOOOO----..
    0x00, 0xfe, 0x7f, 0xc0,  // --------OOOOOOO--OOOOOOOOO----..
    0x01, 0xfc, 0x7f, 0xc0,  // -------OOOOOOO---OOOOOOOOO----..
    0x03, 0xf8, 0x7f, 0xc0,  // ------OOOOOOO----OOOOOOOOO----..
    0x07, 0xf8, 0x7f, 0xc0,  // -----OOOOOOOO----OOOOOOOOO----..
    0x0f, 0xf0, 0x7f, 0xc0,  // ----OOOOOOOO-----OOOOOOOOO----..
    0x0f, 0xe0, 0x7f, 0xc0,  // ----OOOOOOO------OOOOOOOOO----..
    0x1f, 0xc0, 0x7f, 0xc0,  // ---OOOOOOO-------OOOOOOOOO----..
    0x3f, 0x80, 0x7f, 0xc0,  // --OOOOOOO--------OOOOOOOOO----..
    0x7f, 0x80, 0x7f, 0xc0,  // -OOOOOOOO--------OOOOOOOOO----..
    0x7f, 0xff, 0xff, 0xfc,  // -OOOOOOOOOOOOOOOOOOOOOOOOOOOOO..
    0x7f, 0xff, 0xff, 0xfc,  // -OOOOOOOOOOOOOOOOOOOOOOOOOOOOO..
    0x7f, 0xff, 0xff, 0xfc,  // -OOOOOOOOOOOOOOOOOOOOOOOOOOOOO..
    0x7f, 0xff, 0xff, 0xfc,  // -OOOOOOOOOOOOOOOOOOOOOOOOOOOOO..
    0x7f, 0xff, 0xff, 0xfc,  // -OOOOOOOOOOOOOOOOOOOOOOOOOOOOO..
    0x7f, 0xff, 0xff, 0xfc,  // -OOOOOOOOOOOOOOOOOOOOOOOOOOOOO..
    0x7f, 0xff, 0xff, 0xfc,  // -OOOOOOOOOOOOOOOOOOOOOOOOOOOOO..
    0x00, 0x00, 0x7f, 0xc0,  // -----------------OOOOOOOOO----..
    0x00, 0x00, 0x7f, 0xc0,  // -----------------OOOOOOOOO----..
    0x00, 0x00, 0x7f, 0xc0,  // -----------------OOOOOOOOO----..
    0x00, 0x00, 0x7f, 0xc0,  // -----------------OOOOOOOOO----..
    0x00, 0x00, 0x7f, 0xc0,  // -----------------OOOOOOOOO----..
    0x00, 0x00, 0x7f, 0xc0,  // -----------------OOOOOOOOO----..
    0x00, 0x00, 0x00, 0x00,  // ------------------------------..

    // ASCII: 53, char width: 30
    0x00, 0x00, 0x00, 0x00,  // ------------------------------..
    0x03, 0xff, 0xff, 0xc0,  // ------OOOOOOOOOOOOOOOOOOOO----..
    0x03, 0xff, 0xff, 0xc0,  // ------OOOOOOOOOOOOOOOOOOOO----..
    0x03, 0xff, 0xff, 0xc0,  // ------OOOOOOOOOOOOOOOOOOOO----..
    0x07, 0xff, 0xff, 0xc0,  // -----OOOOOOOOOOOOOOOOOOOOO----..
    0x07, 0xff, 0xff, 0xc0,  // -----OOOOOOOOOOOOOOOOOOOOO----..
    0x07, 0xff, 0xff, 0xc0,  // -----OOOOOOOOOOOOOOOOOOOOO----..
    0x07, 0xff, 0xff, 0xc0,  // -----OOOOOOOOOOOOOOOOOOOOO----..
    0x07, 0xf8, 0x00, 0x00,  // -----OOOOOOOO-----------------..
    0x07, 0xf8, 0x00, 0x00,  // -----OOOOOOOO-----------------..
    0x0f, 0xf0, 0x00, 0x00,  // ----OOOOOOOO------------------..
    0x0f, 0xf3, 0xf8, 0x00,  // ----OOOOOOOO--OOOOOOO---------..
    0x0f, 0xff, 0xfe, 0x00,  // ----OOOOOOOOOOOOOOOOOOO-------..
    0x0f, 0xff, 0xff, 0x80,  // ----OOOOOOOOOOOOOOOOOOOOO-----..
    0x0f, 0xff, 0xff, 0xc0,  // ----OOOOOOOOOOOOOOOOOOOOOO----..
    0x1f, 0xff, 0xff, 0xc0,  // ---OOOOOOOOOOOOOOOOOOOOOOO----..
    0x1f, 0xff, 0xff, 0xe0,  // ---OOOOOOOOOOOOOOOOOOOOOOOO---..
    0x1f, 0xf8, 0x7f, 0xe0,  // ---OOOOOOOOOO----OOOOOOOOOO---..
    0x07, 0xe0, 0x3f, 0xf0,  // -----OOOOOO-------OOOOOOOOOO--..
    0x00, 0x00, 0x1f, 0xf0,  // -------------------OOOOOOOOO--..
    0x00, 0x00, 0x1f, 0xf0,  // -------------------OOOOOOOOO--..
    0x00, 0x00, 0x1f, 0xf0,  // -------------------OOOOOOOOO--..
    0x00, 0x00, 0x1f, 0xf0,  // -------------------OOOOOOOOO--..
    0x00, 0x00, 0x1f, 0xf0,  // -------------------OOOOOOOOO--..
    0x03, 0xe0, 0x1f, 0xf0,  // ------OOOOO--------OOOOOOOOO--..
    0x3f, 0xe0, 0x1f, 0xe0,  // --OOOOOOOOO--------OOOOOOOO---..
    0x1f, 0xf0, 0x3f, 0xe0,  // ---OOOOOOOOO------OOOOOOOOO---..
    0x1f, 0xf8, 0x7f, 0xe0,  // ---OOOOOOOOOO----OOOOOOOOOO---..
    0x1f, 0xff, 0xff, 0xc0,  // ---OOOOOOOOOOOOOOOOOOOOOOO----..
    0x0f, 0xff, 0xff, 0xc0,  // ----OOOOOOOOOOOOOOOOOOOOOO----..
    0x07, 0xff, 0xff, 0x80,  // -----OOOOOOOOOOOOOOOOOOOO-----..
    0x03, 0xff, 0xff, 0x00,  // ------OOOOOOOOOOOOOOOOOO------..
    0x00, 0xff, 0xfc, 0x00,  // --------OOOOOOOOOOOOOO--------..
    0x00, 0x3f, 0xf0, 0x00,  // ----------OOOOOOOOOO----------..

    // ASCII: 54, char width: 30
    0x00, 0x1f, 0xf0, 0x00,  // -----------OOOOOOOOO----------..
    0x00, 0x7f, 0xfc, 0x00,  // ---------OOOOOOOOOOOOO--------..
    0x01, 0xff, 0xff, 0x00,  // -------OOOOOOOOOOOOOOOOO------..
    0x03, 0xff, 0xff, 0x80,  // ------OOOOOOOOOOOOOOOOOOO-----..
    0x07, 0xff, 0xff, 0x80,  // -----OOOOOOOOOOOOOOOOOOOO-----..
    0x07, 0xff, 0xff, 0xc0,  // -----OOOOOOOOOOOOOOOOOOOOO----..
    0x0f, 0xfc, 0x7f, 0xc0,  // ----OOOOOOOOOO---OOOOOOOOO----..
    0x0f, 0xf8, 0x7f, 0xe0,  // ----OOOOOOOOO----OOOOOOOOOO---..
    0x1f, 0xf0, 0x3f, 0xe0,  // ---OOOOOOOOO------OOOOOOOOO---..
    0x1f, 0xf0, 0x38, 0x00,  // ---OOOOOOOOO------OOO---------..
    0x1f, 0xf0, 0x00, 0x00,  // ---OOOOOOOOO------------------..
    0x3f, 0xe0, 0x00, 0x00,  // --OOOOOOOOO-------------------..
    0x3f, 0xe1, 0xf8, 0x00,  // --OOOOOOOOO----OOOOOO---------..
    0x3f, 0xe7, 0xfe, 0x00,  // --OOOOOOOOO--OOOOOOOOOO-------..
    0x3f, 0xef, 0xff, 0x80,  // --OOOOOOOOO-OOOOOOOOOOOOO-----..
    0x3f, 0xff, 0xff, 0x80,  // --OOOOOOOOOOOOOOOOOOOOOOO-----..
    0x3f, 0xff, 0xff, 0xc0,  // --OOOOOOOOOOOOOOOOOOOOOOOO----..
    0x3f, 0xff, 0xff, 0xe0,  // --OOOOOOOOOOOOOOOOOOOOOOOOO---..
    0x3f, 0xf8, 0x7f, 0xe0,  // --OOOOOOOOOOO----OOOOOOOOOO---..
    0x3f, 0xf0, 0x3f, 0xf0,  // --OOOOOOOOOO------OOOOOOOOOO--..
    0x3f, 0xf0, 0x1f, 0xf0,  // --OOOOOOOOOO-------OOOOOOOOO--..
    0x3f, 0xe0, 0x1f, 0xf0,  // --OOOOOOOOO--------OOOOOOOOO--..
    0x3f, 0xe0, 0x1f, 0xf0,  // --OOOOOOOOO--------OOOOOOOOO--..
    0x1f, 0xe0, 0x1f, 0xf0,  // ---OOOOOOOO--------OOOOOOOOO--..
    0x1f, 0xe0, 0x1f, 0xf0,  // ---OOOOOOOO--------OOOOOOOOO--..
    0x1f, 0xf0, 0x1f, 0xf0,  // ---OOOOOOOOO-------OOOOOOOOO--..
    0x0f, 0xf0, 0x3f, 0xe0,  // ----OOOOOOOO------OOOOOOOOO---..
    0x0f, 0xf8, 0x7f, 0xe0,  // ----OOOOOOOOO----OOOOOOOOOO---..
    0x07, 0xff, 0xff, 0xe0,  // -----OOOOOOOOOOOOOOOOOOOOOO---..
    0x07, 0xff, 0xff, 0xc0,  // -----OOOOOOOOOOOOOOOOOOOOO----..
    0x03, 0xff, 0xff, 0x80,  // ------OOOOOOOOOOOOOOOOOOO-----..
    0x01, 0xff, 0xff, 0x00,  // -------OOOOOOOOOOOOOOOOO------..
    0x00, 0x7f, 0xfe, 0x00,  // ---------OOOOOOOOOOOOOO-------..
    0x00, 0x1f, 0xf0, 0x00,  // -----------OOOOOOOOO----------..

    // ASCII: 55, char width: 30
    0x00, 0x00, 0x00, 0x00,  // ------------------------------..
    0x3f, 0xff, 0xff, 0xf0,  // --OOOOOOOOOOOOOOOOOOOOOOOOOO--..
    0x3f, 0xff, 0xff, 0xf0,  // --OOOOOOOOOOOOOOOOOOOOOOOOOO--..
    0x3f, 0xff, 0xff, 0xf0,  // --OOOOOOOOOOOOOOOOOOOOOOOOOO--..
    0x3f, 0xff, 0xff, 0xf0,  // --OOOOOOOOOOOOOOOOOOOOOOOOOO--..
    0x3f, 0xff, 0xff, 0xf0,  // --OOOOOOOOOOOOOOOOOOOOOOOOOO--..
    0x3f, 0xff, 0xff, 0xf0,  // --OOOOOOOOOOOOOOOOOOOOOOOOOO--..
    0x3f, 0xff, 0xff, 0xe0,  // --OOOOOOOOOOOOOOOOOOOOOOOOO---..
    0x00, 0x00, 0x7f, 0xc0,  // -----------------OOOOOOOOO----..
    0x00, 0x00, 0xff, 0x80,  // ----------------OOOOOOOOO-----..
    0x00, 0x00, 0xff, 0x00,  // ----------------OOOOOOOO------..
    0x00, 0x01, 0xfe, 0x00,  // ---------------OOOOOOOO-------..
    0x00, 0x03, 0xfe, 0x00,  // --------------OOOOOOOOO-------..
    0x00, 0x03, 0xfc, 0x00,  // --------------OOOOOOOO--------..
    0x00, 0x07, 0xf8, 0x00,  // -------------OOOOOOOO---------..
    0x00, 0x0f, 0xf8, 0x00,  // ------------OOOOOOOOO---------..
    0x00, 0x0f, 0xf0, 0x00,  // ------------OOOOOOOO----------..
    0x00, 0x1f, 0xf0, 0x00,  // -----------OOOOOOOOO----------..
    0x00, 0x1f, 0xe0, 0x00,  // -----------OOOOOOOO-----------..
    0x00, 0x3f, 0xe0, 0x00,  // ----------OOOOOOOOO-----------..
    0x00, 0x3f, 0xe0, 0x00,  // ----------OOOOOOOOO-----------..
    0x00, 0x3f, 0xc0, 0x00,  // ----------OOOOOOOO------------..
    0x00, 0x7f, 0xc0, 0x00,  // ---------OOOOOOOOO------------..
    0x00, 0x7f, 0xc0, 0x00,  // ---------OOOOOOOOO------------..
    0x00, 0x7f, 0x80, 0x00,  // ---------OOOOOOOO-------------..
    0x00, 0xff, 0x80, 0x00,  // --------OOOOOOOOO-------------..
    0x00, 0xff, 0x80, 0x00,  // --------OOOOOOOOO-------------..
    0x00, 0xff, 0x80, 0x00,  // --------OOOOOOOOO-------------..
    0x00, 0xff, 0x80, 0x00,  // --------OOOOOOOOO-------------..
    0x00, 0xff, 0x00, 0x00,  // --------OOOOOOOO--------------..
    0x01, 0xff, 0x00, 0x00,  // -------OOOOOOOOO--------------..
    0x01, 0xff, 0x00, 0x00,  // -------OOOOOOOOO--------------..
    0x01, 0xff, 0x00, 0x00,  // -------OOOOOOOOO--------------..
    0x00, 0x00, 0x00, 0x00,  // ------------------------------..

    // ASCII: 56, char width: 30
    0x00, 0x3f, 0xf0, 0x00,  // ----------OOOOOOOOOO----------..
    0x01, 0xff, 0xfe, 0x00,  // -------OOOOOOOOOOOOOOOO-------..
    0x03, 0xff, 0xff, 0x00,  // ------OOOOOOOOOOOOOOOOOO------..
    0x07, 0xff, 0xff, 0x80,  // -----OOOOOOOOOOOOOOOOOOOO-----..
    0x0f, 0xff, 0xff, 0xc0,  // ----OOOOOOOOOOOOOOOOOOOOOO----..
    0x0f, 0xff, 0xff, 0xc0,  // ----OOOOOOOOOOOOOOOOOOOOOO----..
    0x1f, 0xf8, 0x7f, 0xe0,  // ---OOOOOOOOOO----OOOOOOOOOO---..
    0x1f, 0xf0, 0x3f, 0xe0,  // ---OOOOOOOOO------OOOOOOOOO---..
    0x1f, 0xf0, 0x3f, 0xe0,  // ---OOOOOOOOO------OOOOOOOOO---..
    0x1f, 0xf0, 0x3f, 0xe0,  // ---OOOOOOOOO------OOOOOOOOO---..
    0x1f, 0xf0, 0x3f, 0xe0,  // ---OOOOOOOOO------OOOOOOOOO---..
    0x0f, 0xf8, 0x7f, 0xc0,  // ----OOOOOOOOO----OOOOOOOOO----..
    0x0f, 0xff, 0xff, 0xc0,  // ----OOOOOOOOOOOOOOOOOOOOOO----..
    0x07, 0xff, 0xff, 0x80,  // -----OOOOOOOOOOOOOOOOOOOO-----..
    0x01, 0xff, 0xff, 0x00,  // -------OOOOOOOOOOOOOOOOO------..
    0x03, 0xff, 0xff, 0x00,  // ------OOOOOOOOOOOOOOOOOO------..
    0x07, 0xff, 0xff, 0x80,  // -----OOOOOOOOOOOOOOOOOOOO-----..
    0x0f, 0xff, 0xff, 0xc0,  // ----OOOOOOOOOOOOOOOOOOOOOO----..
    0x1f, 0xf8, 0x7f, 0xe0,  // ---OOOOOOOOOO----OOOOOOOOOO---..
    0x1f, 0xf0, 0x3f, 0xe0,  // ---OOOOOOOOO------OOOOOOOOO---..
    0x3f, 0xe0, 0x3f, 0xf0,  // --OOOOOOOOO-------OOOOOOOOOO--..
    0x3f, 0xe0, 0x1f, 0xf0,  // --OOOOOOOOO--------OOOOOOOOO--..
    0x3f, 0xe0, 0x1f, 0xf0,  // --OOOOOOOOO--------OOOOOOOOO--..
    0x3f, 0xe0, 0x1f, 0xf0,  // --OOOOOOOOO--------OOOOOOOOO--..
    0x3f, 0xe0, 0x1f, 0xf0,  // --OOOOOOOOO--------OOOOOOOOO--..
    0x3f, 0xf0, 0x3f, 0xf0,  // --OOOOOOOOOO------OOOOOOOOOO--..
    0x3f, 0xf0, 0x3f, 0xf0,  // --OOOOOOOOOO------OOOOOOOOOO--..
    0x1f, 0xf8, 0x7f, 0xe0,  // ---OOOOOOOOOO----OOOOOOOOOO---..
    0x1f, 0xff, 0xff, 0xe0,  // ---OOOOOOOOOOOOOOOOOOOOOOOO---..
    0x0f, 0xff, 0xff, 0xc0,  // ----OOOOOOOOOOOOOOOOOOOOOO----..
    0x07, 0xff, 0xff, 0x80,  // -----OOOOOOOOOOOOOOOOOOOO-----..
    0x03, 0xff, 0xff, 0x00,  // ------OOOOOOOOOOOOOOOOOO------..
    0x01, 0xff, 0xfe, 0x00,  // -------OOOOOOOOOOOOOOOO-------..
    0x00, 0x3f, 0xf0, 0x00,  // ----------OOOOOOOOOO----------..

    // ASCII: 57, char width: 30
    0x00, 0x3f, 0xe0, 0x00,  // ----------OOOOOOOOO-----------..
    0x01, 0xff, 0xf8, 0x00,  // -------OOOOOOOOOOOOOO---------..
    0x03, 0xff, 0xfe, 0x00,  // ------OOOOOOOOOOOOOOOOO-------..
    0x07, 0xff, 0xff, 0x00,  // -----OOOOOOOOOOOOOOOOOOO------..
    0x0f, 0xff, 0xff, 0x80,  // ----OOOOOOOOOOOOOOOOOOOOO-----..
    0x1f, 0xff, 0xff, 0x80,  // ---OOOOOOOOOOOOOOOOOOOOOO-----..
    0x1f, 0xf8, 0x7f, 0xc0,  // ---OOOOOOOOOO----OOOOOOOOO----..
    0x1f, 0xf0, 0x3f, 0xc0,  // ---OOOOOOOOO------OOOOOOOO----..
    0x3f, 0xe0, 0x3f, 0xe0,  // --OOOOOOOOO-------OOOOOOOOO---..
    0x3f, 0xe0, 0x1f, 0xe0,  // --OOOOOOOOO--------OOOOOOOO---..
    0x3f, 0xe0, 0x1f, 0xe0,  // --OOOOOOOOO--------OOOOOOOO---..
    0x3f, 0xe0, 0x1f, 0xf0,  // --OOOOOOOOO--------OOOOOOOOO--..
    0x3f, 0xe0, 0x1f, 0xf0,  // --OOOOOOOOO--------OOOOOOOOO--..
    0x3f, 0xe0, 0x3f, 0xf0,  // --OOOOOOOOO-------OOOOOOOOOO--..
    0x3f, 0xf0, 0x3f, 0xf0,  // --OOOOOOOOOO------OOOOOOOOOO--..
    0x1f, 0xf8, 0x7f, 0xf0,  // ---OOOOOOOOOO----OOOOOOOOOOO--..
    0x1f, 0xff, 0xff, 0xf0,  // ---OOOOOOOOOOOOOOOOOOOOOOOOO--..
    0x0f, 0xff, 0xff, 0xf0,  // ----OOOOOOOOOOOOOOOOOOOOOOOO--..
    0x07, 0xff, 0xff, 0xf0,  // -----OOOOOOOOOOOOOOOOOOOOOOO--..
    0x07, 0xff, 0xdf, 0xf0,  // -----OOOOOOOOOOOOO-OOOOOOOOO--..
    0x01, 0xff, 0x9f, 0xf0,  // -------OOOOOOOOOO--OOOOOOOOO--..
    0x00, 0x7e, 0x1f, 0xf0,  // ---------OOOOOO----OOOOOOOOO--..
    0x00, 0x00, 0x1f, 0xf0,  // -------------------OOOOOOOOO--..
    0x00, 0x00, 0x3f, 0xe0,  // ------------------OOOOOOOOO---..
    0x00, 0x70, 0x3f, 0xe0,  // ---------OOO------OOOOOOOOO---..
    0x1f, 0xf0, 0x3f, 0xe0,  // ---OOOOOOOOO------OOOOOOOOO---..
    0x1f, 0xf8, 0x7f, 0xc0,  // ---OOOOOOOOOO----OOOOOOOOO----..
    0x0f, 0xf8, 0xff, 0xc0,  // ----OOOOOOOOO---OOOOOOOOOO----..
    0x0f, 0xff, 0xff, 0x80,  // ----OOOOOOOOOOOOOOOOOOOOO-----..
    0x07, 0xff, 0xff, 0x80,  // -----OOOOOOOOOOOOOOOOOOOO-----..
    0x03, 0xff, 0xff, 0x00,  // ------OOOOOOOOOOOOOOOOOO------..
    0x03, 0xff, 0xfe, 0x00,  // ------OOOOOOOOOOOOOOOOO-------..
    0x00, 0xff, 0xf8, 0x00,  // --------OOOOOOOOOOOOO---------..
    0x00, 0x3f, 0xe0, 0x00,  // ----------OOOOOOOOO-----------..
};

const struct font_t font34 PROGMEM = {
   32,  // width in pixels (must be multiple of 8)
   34,  // height in pixels
   {
      XX, XX, XX, XX, XX, XX, XX, XX, XX, XX, XX, XX, XX, XX, XX, XX, // space to /
      1,  2,  3,  4,  5,  6,  7,  8,  9,  10, XX, XX, XX, XX, XX, XX, // 0 to ?
      XX, XX, XX, XX, XX, XX, XX, XX, XX, XX, XX, XX, XX, XX, XX, XX, // @ to O
      XX, XX, XX, XX, XX, XX, XX, XX, XX, XX, XX, XX, XX, XX, XX, XX, // P to _
      00, XX, XX, XX, XX, XX, XX, XX, XX, XX, XX, XX, XX, XX, XX, XX, // ` (used for degree symbol) to o
      XX, XX, XX, XX, XX, XX, XX, XX, XX, XX, XX, XX, XX, XX, XX },   // p to ~
   &font34_data[0] };

//* font34.cpp
