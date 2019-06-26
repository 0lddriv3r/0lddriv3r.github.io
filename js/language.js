function translate(){
    var value = sessionStorage.getItem("language");
    if(value==="1"){
        sessionStorage.setItem("language", "0"); 
    }else{
        sessionStorage.setItem("language", "1");
    }
    window.location.reload();//refresh the webpage.
}