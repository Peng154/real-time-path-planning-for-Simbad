package com;

import simbad.gui.Simbad;
import com.base.Env;

public class Execute {


    public static void main(String[] args) {
        // request antialising
        System.setProperty("j3d.implicitAntialiasing", "true");
        // create Simbad instance with given environment
        Simbad frame = new Simbad(new Env(), false);
    }

}

