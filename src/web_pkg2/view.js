//Table
var row_okey;
var tagsArray;
var selected_row;
var array_robots = [];
var topic_selected={name:"",type:"",date_init:""};

window.onload = function () {
    this.console.log("[view.js] View loaded ")

   
}


function filloutTable(array/** topic names */, array2/** topic type */) {

}



function getCurrentDate(){
    let  d = new Date();
    let date_format= ("0" + d.getDate()).slice(-2) + "-" + ("0"+(d.getMonth()+1)).slice(-2) + "-" +
    d.getFullYear() + " " + ("0" + d.getHours()).slice(-2) + ":" + ("0" + d.getMinutes()).slice(-2) +":" + ("0" + d.getSeconds()).slice(-2);
    return date_format;
}

function addViewNewSubs(objRos){
    let {name, messageType} =objRos;
    let simplename=getNodeName_fromNameTopic(name);
    console.log({simplename})
    let id1="x";let id2="y"
        let sus_names='<div id="'+simplename+'"><p> X '+simplename+' pose:</p>'+
                      '<p>Y '+simplename+' pose:</p></div>'
        //var para = document.createElement(sus_names);

        document.getElementById("subs_names").innerHTML += sus_names;
     
 }

 function getNodeName_fromNameTopic(name){
    if(name)return name.split("/")[1]
    else return null;
 }