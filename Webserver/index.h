const char MAIN_page[] PROGMEM = R"=====(
<!DOCTYPE html>
<html>
<style>
.card{
    max-width: 400px;
     min-height: 250px;
     background: #007bb3;
     padding: 30px;
     box-sizing: border-box;
     color: #FFF;
     margin:20px;
     box-shadow: 0px 2px 18px -4px rgba(0,0,0,0.75);
}
</style>
<body>

<div class="card">
  <h4>BitForce Status Page</h4><br>
  <h1>Counter Value:<span id="CounterValue">N/A</span></h1><br>
  <br><a href="https://circuits4you.com">Circuits4you.com</a>
</div>
<script>

setInterval(function() {
  // Call a function repetatively with 500 ms interval
  getData();
}, 500);

function getData() {
  var xhttp = new XMLHttpRequest();
  xhttp.onreadystatechange = function() {
    if (this.readyState == 4 && this.status == 200) {
      document.getElementById("CounterValue").innerHTML =
      this.responseText;
    }
  };
  xhttp.open("GET", "updateStatus", true);
  xhttp.send();
}
</script>
</body>
</html>
)=====";
