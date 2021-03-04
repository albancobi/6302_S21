/*
Default Slider module:
div_id: The div_id to which you want to attach the slider
label: The label (title) of the slider
min: The minimum value you want the slider to be able to output
max: The maximum value you want the slider to be able to output
resolution: The resolution/granularity of the output values
unique: A unique (for the GUI) phrase used in DOM construction to prevent cross-talk between event handling
color: (default null): Unsupported right now
socket: (default null): a standard websocket object for outside communication emission
*/

function Slider(uniqueIn,labelIn,minIn,maxIn,resIn,color=null){
    var unique = String(uniqueIn); //unique identifying number
    var div_id = "box_" + unique;
    var label = String(labelIn);
    var color = color;
    var overall_div = document.getElementById(div_id);
    var holder;
    var min = parseFloat(minIn);
    var max = parseFloat(maxIn);
    var resolution = parseFloat(resIn);
    var slider_element;
    var spec_input;
    var total_element;
    var spec_holder;
    var spec_label;
    var val_holder;
    var event = new Event('change');

    var setup = function(){
        //var handle = document.createElement("div");
        //handle.setAttribute("class","handle");
        holder = document.createElement("div");
        holder.setAttribute("id", div_id+unique+"_holder");
        holder.setAttribute("class", "slider_holder");
        //holder.appendChild(handle);
        overall_div.appendChild(holder);
        var label_element = document.createElement("div");
        label_element.setAttribute("class","slider_label handle");
        label_element.innerHTML = label;
        holder.appendChild(label_element);
        slider_element = document.createElement("div");
        slider_element.setAttribute("id", div_id+unique+"slider");
        holder.appendChild(slider_element); 
        val_holder = document.createElement("div");
        val_holder.setAttribute("class","slider_values_holder");
        spec_holder = document.createElement("div");
        spec_label = document.createElement("div");
        spec_label.innerHTML = "Set Value:";
        spec_holder.setAttribute("class","slider_input_holder");
        spec_input = document.createElement("input");
        spec_input.setAttribute("type","number");
        spec_input.setAttribute("step",resolution);
        spec_input.setAttribute("min",min);
        spec_input.setAttribute("max",max);
        spec_input.setAttribute("id",div_id+unique+"manual_input");
        spec_input.setAttribute("class","numerical_input");
        spec_holder.appendChild(spec_label);
        spec_holder.appendChild(spec_input);
        //var inlabel = document.createElement("span");
        //inlabel.innerHTML= "Value:";
        //holder.appendChild(inlabel);
        holder.appendChild(val_holder);
        val_holder.appendChild(spec_holder);
        noUiSlider.create(slider_element, {
            start: 0.0,
            connect: true,
            range: {
                'min': min,
                'max': max
            }
        });
        slider_element.noUiSlider.on('update',throttle(function(value) {
            spec_input.value = value;
            var local_change =
		new CustomEvent('ui_change',
				{detail:{"message":unique+":"+String(value)}});
            document.dispatchEvent(local_change);              
        },10));
    }
    setup();

    this.update = function(value){
        slider_element.noUiSlider.set([parseFloat(value)]); //update value except for limits 
    };

    this.value = function(){
        return parseFloat(spec_input.value);
    }
    spec_input.addEventListener('click', function(){
	slider_element.noUiSlider.set([this.value]);

    });
};





