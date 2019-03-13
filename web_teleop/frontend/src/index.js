"use strict";

$(function(){
    $(document).ready(function() {
        
        // switch between auto mode and manual mode
        $("#auto_btn").click(function() {
            $("#auto").css({"display":"block"});
            $("#manual").css({"display":"none"});
        });
        $("#manual_btn").click(function() {
            $("#auto").css({"display":"none"});
            $("#manual").css({"display":"block"});
        });
        
        // SLIDER
        // var slider = document.getElementById("myRange");
        // var output = document.getElementById("demo");
        // output.innerHTML = slider.value; // Display the default slider value
        
        // // Update the current slider value (each time you drag the slider handle)
        // slider.oninput = function() {
        //   output.innerHTML = this.value;
        // }
        // SLIDER
    });
});
