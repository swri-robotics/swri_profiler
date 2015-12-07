var stringHashCode = function(s){
	var hash = 0;
	if (s.length == 0) return hash;
	for (i = 0; i < s.length; i++) {
		char = s.charCodeAt(i);
		hash = ((hash<<5)-hash)+char;
		hash = hash & hash; // Convert to 32bit integer
	}
	return hash;
};

var formatDuration = function(ns) {
  if (ns < 1e3) { return ns + "ns"; }
  else if (ns < 1e6) { return (ns / 1e3).toFixed(3) + "us"; } 
  else if (ns < 1e9) { return (ns / 1e6).toFixed(3) + "ms"; }

  var s = ns / 1e9;
  if (s < 60.0) { 
    return s.toFixed(3) + "s";
  }

  var m = Math.floor(s / 60.0);
  s = s % 60.0;
  if (m < 60) {
    return m + "m " + s.toFixed(1) + "s";
  }

  // If we get to hours, we stop caring about fractional seconds.
  s = Math.round(s)  
  
  var h = Math.floor(m / 60);
  m = m % 60;
  if (h < 24) {
    return h + "h " + m + "m " + s + "s";
  }

  var d = Math.floor(h / 24);
  h = h % 24
  if (d < 365) {
    return d + "d " + h + "h " + m + "m " + s + "s";
  }

  var y = Math.floor(d / 365);
  d = d % 24;
  return y + "y" + d + "d" + h + "h" + m + "m" + s + "s";
}

var ProfileNode = function(rel_name) {
  this.rel_name = rel_name;
  this.abs_name = null;
  this.measured = false;
  this.internal = false;
  this.call_count = null;
  this.total_duration = null;
  this.max_duration = null;
  this.children = [];
}

ProfileNode.prototype.rebuild = function(parent_abs_name) {
  if (parent_abs_name == null) {
    this.abs_name = "";
  } else {
    this.abs_name = parent_abs_name + '/' + this.rel_name;
  }
  var hash = stringHashCode(this.abs_name);
  this.color = d3.rgb(hash & 0xFF, (hash >> 8) & 0xFF, (hash >> 16) & 0xFF);

  var children_total_duration = 0;

  if (this.hasOwnProperty('children')) {
    for (var i = 0; i < this.children.length; i++) {
      if (this.children[i].internal) {
        continue;
      }
      this.children[i].rebuild(this.abs_name);
      children_total_duration += this.children[i].total_duration;
    }
  }

  if (!this.measured) {
    this.total_duration = children_total_duration;
  }

  if (this.hasOwnProperty('children') && this.children.length > 0) {
    var internal = this.getChild("[internal]");
    internal.internal = true;
    // internal.color = this.color;
    internal.abs_name = this.abs_name + " [internal]";
    internal.total_duration = Math.max(0, this.total_duration - children_total_duration);
  }
};

ProfileNode.prototype.totalDuration = function() {
  return this.total_duration;
};

ProfileNode.prototype.callCount = function() {
  return this.call_count;
}

ProfileNode.prototype.findChildIndex = function(rel_name) {
  if (!this.hasOwnProperty('children')) {
    this.children = [];
    return -1;
  }

  for (var i = 0; i < this.children.length; i++) {
    if (this.children[i].rel_name == rel_name) {
      return i;
    }
  }
  return -1;
};

// Get or create the child node with a given leaf name.
ProfileNode.prototype.getChild = function(rel_name) {
  var index = this.findChildIndex(rel_name);
  if (index >= 0) { 
    return this.children[index];
  } else {
    var new_child = new ProfileNode(rel_name);
    this.children.push(new_child);
    return new_child;
  }    
};

function stringStartsWith (string, prefix) {
    return string.slice(0, prefix.length) == prefix;
}
// Trims all leading and trailing slashes from a string.
var trimSlash = function(s) {
  return s.replace(/^\/*|\/*$/g, "");
}

// Convert a duration object to integer number of nanoseconds.
var toNSec = function(duration) {
    return duration.secs*1e9 + duration.nsecs;
}

var handleProfilerIndex = function(msg) {
  var node_name = trimSlash(msg.header.frame_id);
  console.log('Received new index for ' + node_name);
  index = {}
  for (var i = 0; i < msg.data.length; i++) {
    index[msg.data[i].key] = trimSlash(msg.data[i].label);
  }
  profile_indices[node_name] = index;
};

var handleProfilerData = function(msg) {
  var node_name = trimSlash(msg.header.frame_id);
  if (!(node_name in profile_indices)) {
    console.log('Dropping profile data for node without index: ' + node_name);
    return;
  }

  var fullNameFromKey = function(key) {
    var node_index = profile_indices[node_name];
    if (!(key in node_index)) {
      console.log('Missing key ' + key + ' in index for node ' + node_name);
      return undefined;
    }
   
    var label = node_index[key];

    // This is a special case to handle nodelets nicely, and it
    // requires that our users design their labels intelligently by
    // wrapping each ROS callback with a Profiler
    // nlet_prof(getName()).  If the nodelet is run as part of a
    // nodelet manager, the node name and nodelet name will differ and
    // we want to append the node name to allow us to guage the
    // relative runtimes of all the instrumented nodelets in that
    // manager.  If the nodelet is run standalone, then the nodelet
    // name and node name will be the same and we don't need to
    // duplicate it.
    if (!stringStartsWith(label, node_name)) {
      label = node_name + "/" + label;
    }

    // Note that label is guaranteed to start with the ROS node's
    // name, without a leading slash.
    return trimSlash(label);
  }

  var handleProfileItem = function(item) {
    var full_name = fullNameFromKey(item.key);
    if (!full_name) {
      return;
    }

    var parts = full_name.split('/');
    var node = profile_root;
    for (var i = 0; i < parts.length; i++) {
      node = node.getChild(parts[i]);
    }
    node.measured = true;
    node.call_count = item.call_count;
    node.total_duration = toNSec(item.total_duration);
    node.max_duration = toNSec(item.abs_max_duration);
  }

  for (var i = 0; i < msg.data.length; i++) {
    handleProfileItem(msg.data[i]);
  }  

  // Rebuild the tree to update total_duration for everyone.
  profile_root.rebuild();
  
  // updateCanvas();
};

var showTooltip = function(d) {
      var tooltip = d3.select("#tooltip");
      tooltip.style("left", d3.event.pageX + "px")
        .style("top", d3.event.pageY + "px");
  tooltip.select("#label").text(d.abs_name);
  tooltip.select("#calls").text(d.measured ? d.call_count : "N/A");
  tooltip.select("#total_duration").text(formatDuration(d.total_duration));  
  tooltip.select("#avg_duration").text(
    d.measured && d.call_count > 0 ? formatDuration(d.total_duration / d.call_count) : "N/A");  
  tooltip.select("#max_duration").text(d.measured ? formatDuration(d.max_duration) : "N/A");

  d3.select("#tooltip").classed("hidden", false);
}

var hideTooltip = function() {
  d3.select("#tooltip").classed("hidden", true);
}

var updateCanvasArc = function() {
  partition.size([2 * Math.PI, burst_radius * burst_radius]);

  var startParams = function(d) {
    return {
      startAngle: d.x + 0.9*d.dx/2.0,
      endAngle:   d.x + 1.1*d.dx/2.0,
      innerRadius: Math.sqrt(d.y),
      outerRadius: Math.sqrt(d.y + d.dy)
    };
  }

  var finalParams = function (d) {
    return {
      startAngle: d.x,
      endAngle: d.x + d.dx,
      innerRadius: Math.sqrt(d.y),
      outerRadius: Math.sqrt(d.y + d.dy)
    };
  }      

  var arc = d3.svg.arc();

  var paths = svg.selectAll("path")
    .data(partition.nodes(profile_root), function (d) { return d.abs_name; });

  paths.enter()
    .append("path")
    .attr("display", function(d) { 
      // Arc is having display issues for tiny tiny slivers of data
      // data, so we'll just hide them.  In the future, it'd be cool
      // to group everything below a threshold into a shared node.
      if (d.depth == 0 || d.dx < 0.00001 || d.internal) {
        return "none";
      } else {
        return null;
      }
    })
    .style("stroke", "#fff")
    .attr("fill", function(d, i) { return d.color; })
    .on("mouseover", showTooltip)
    .on("mouseout", hideTooltip)
    .each(function (d) { this._current_arc = startParams(d); });

  paths
    .transition()
    .duration(500)
    .attrTween("d", function(d, i) { 
      var i = d3.interpolateObject(this._current_arc, finalParams(d));
      this._current_arc = finalParams(d);
      return function(t) {
        return arc(i(t));
      };
    });
  var timeout = setTimeout(updateCanvasArc, 500);
};

var updateCanvasRect = function() {
  var x_scale = d3.scale.linear()
    .domain([0.0, 1.0])
    .range([-svg_width/2.0, svg_width/2.0]);
  
  var y_scale = d3.scale.linear()
    .domain([0.0,1.0])
    .range([-svg_height/2.0, svg_height/2.0]);

  var rects = svg.selectAll("rect")
    .data(partition.nodes(profile_root), function (d) { return d.abs_name; });

  rects.enter()
    .append("rect")
    .attr("display", function(d) { 
      // Arc is having display issues for tiny tiny slivers of data
      // data, so we'll just hide them.  In the future, it'd be cool
      // to group everything below a threshold into a shared node.
      if (d.depth == 0 || d.dx < 0.001 || d.internal) {
        return "none";
      } else {
        return null;
      }
    })
    .attr("x", function(d) { return x_scale(d.y); })
    .attr("y", function(d) { return y_scale(d.x); })
    .attr("width", function(d) { return Math.abs(x_scale(d.y) - x_scale(d.y+d.dy)); })
    .attr("height", function(d) { return 0; })
    .style("stroke", "#fff")
    .attr("fill", function(d, i) { return d.color; })
    .on("mouseover", showTooltip)
    .on("mouseout", hideTooltip);

  rects
    .transition()
    .duration(200)
    .attr("x", function(d) { return x_scale(d.y); })
    .attr("y", function(d) { return y_scale(d.x); })
    .attr("width", function(d) { return Math.abs(x_scale(d.y) - x_scale(d.y+d.dy)); })
    .attr("height", function(d) { return Math.abs(y_scale(d.x) - y_scale(d.x+d.dx)); })
          
  var timeout = setTimeout(updateCanvasRect, 500);
};


var svg_width = 400;
var svg_height = 400;
var burst_radius = Math.min(svg_width, svg_height) / 2;

var svg = d3.select("body").append("svg")
    .attr("width", svg_width)
    .attr("height", svg_height)
    .append("g")
    .attr("transform", "translate(" + svg_width / 2 + "," + svg_height / 2 + ")");

var partition = d3.layout.partition()
  //.sort(null)
  .sort(function(a,b) { 
    if (a.internal) { return -1; } 
    else if (b.internal) { return 1; }
    else return d3.ascending(a.abs_name, b.abs_name); })
  .value(function (d) { return d.totalDuration(); });


// Map of node names -> (map of integer labels -> string labels)
var profile_indices = {};
var profile_root = new ProfileNode("");
profile_root.rebuild();

if (true) {
  updateCanvasRect();
} else {
  updateCanvasArc();
}

var ros = new ROSLIB.Ros({url : 'ws://localhost:9091'});
ros.on('connection', function() { console.log('Connected to websocket server.'); });
ros.on('error', function(error) { console.log('Error connecting to websocket server: ', error); });
ros.on('close', function() { console.log('Connection to websocket server closed.'); });      

var index_sub = new ROSLIB.Topic({
  ros : ros,
  name : '/profiler/index',
  messageType : 'swri_profiler_msgs/ProfileIndexArray'
});
index_sub.subscribe(this.handleProfilerIndex);

var data_sub = new ROSLIB.Topic({
  ros : ros,
  name : '/profiler/data',
  messageType : 'swri_profiler_msgs/ProfileDataArray'
});
data_sub.subscribe(this.handleProfilerData);
