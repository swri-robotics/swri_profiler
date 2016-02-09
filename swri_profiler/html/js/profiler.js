////////////////////////////////////////////////////////////////////////////////
// Utility Functions ///////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////
"use strict";

var escapeRegExp = function(s) {
    return s.replace(/[-\/\\^$*+?.()|[\]{}]/g, '\\$&');
};

var stringHashCode = function(s){
  var hash = 0;
  if (s.length == 0) return hash;
  for (var i = 0; i < s.length; i++) {
    var ch = s.charCodeAt(i);
    hash = ((hash<<5)-hash)+ch;
    hash = hash & hash; // Convert to 32bit integer
  }
  return hash;
};

var nullMin = function(a,b) {
  if (a == null && b == null) { return null; }
  else if (a == null) { return b; }
  else if (b == null) { return a; }
  else return Math.min(a,b);
}

var nullMax = function(a,b) {
  if (a == null && b == null) { return null; }
  else if (a == null) { return b; }
  else if (b == null) { return a; }
  else return Math.max(a,b);
}

var formatDuration = function(ns) {
  if (ns == 0) { return "0"; }
  
  if (ns < 1e3) { return ns + "ns"; }
  if (ns < 1e6) { return (ns / 1e3).toFixed(3) + "us"; } 
  if (ns < 1e9) { return (ns / 1e6).toFixed(3) + "ms"; }

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
  return y + "y " + d + "d " + h + "h " + m + "m " + s + "s";
}

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

// Returns the index where the specified time should be entered to
// maintain ordering.
var findTime = function(timeline, t) {
  if (timeline.length == 0) { 
    return 0;
  } 

  // todo: replace with binary/interpolation search.
  for (var i = timeline.length; i > 0 ; i--) {
    if (t > timeline[i-1]) {
      return i;
    } 
  }

  return 0;
}

////////////////////////////////////////////////////////////////////////////////
// Ros Profiler Adapter Object /////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////

// The RosProfilerAdapter subscribes to the profiler topics and
// "uncompresses" the profiler data messages by replacing the integer
// key with the string label for each profiled block.  The
// uncompressed data is passed to a data handler function.

var ROS_BLOCK = false;
var RosProfilerAdapter = function(ros) {
  // Map of node names -> (map of integer labels -> string labels)
  this.indices = {};

  // Default data handler just logs to the console.
  this.data_handler = function(node_name, t, msg) { 
    console.log("Data for " + node_name + " at " + formatDuration(t));
    console.log(items);
  };

  this.index_sub = new ROSLIB.Topic({
    ros : ros,
    name : '/profiler/index',
    messageType : 'swri_profiler_msgs/ProfileIndexArray'
  });
  this.index_sub.subscribe(this.handleIndex.bind(this));

  this.data_sub = new ROSLIB.Topic({
    ros : ros,
    name : '/profiler/data',
    messageType : 'swri_profiler_msgs/ProfileDataArray'
  });
  this.data_sub.subscribe(this.handleData.bind(this));

  return this;
}

// Set/get the data handler for the adapter.
RosProfilerAdapter.prototype.dataHandler = function(f) {
  if (f == undefined) {
    return this.data_handler;
  } else {
    this.data_handler = f;
  }
}

// Close the adapter's subscriptions and release the data_handler
// reference.
RosProfilerAdapter.prototype.close = function() {
  this.index_sub.unsubscribe();
  this.data_sub.unsubscribe();
  this.data_handler = function(node_name, t, msg) {};
}

RosProfilerAdapter.prototype.handleIndex = function(msg) {
  if (ROS_BLOCK) { return; };

  var node_name = trimSlash(msg.header.frame_id);
  console.log('Received new index for ' + node_name);
  var index = {}
  for (var i = 0; i < msg.data.length; i++) {
    index[msg.data[i].key] = trimSlash(msg.data[i].label);
  }
  this.indices[node_name] = index;
};

RosProfilerAdapter.prototype.handleData = function(msg) {
  if (ROS_BLOCK) { return; };

  var node_name = trimSlash(msg.header.frame_id);
  if (!(node_name in this.indices)) {
    console.log('Dropping profile data for node without index: ' + node_name);
    return;
  }

  // SWRI_PROFILER tries to publish data aligned to every second.  We
  // can then round them to the nearest second without affecting the
  // report noticeably, which gives us a nice uniform timebase for all
  // nodes to share.
  var secs = Math.round(toNSec(msg.header.stamp)/1e9);

  var node_index = this.indices[node_name];

  var fullNameFromKey = function(key) {
    if (!(key in node_index)) {
      console.log('Missing key ' + key + ' in index for node ' + node_name);
      return undefined;
    }
   
    var label = node_index[key];

    // This is a special case to handle nodelets nicely, and it works
    // when users design their labels intelligently by wrapping each
    // ROS callback with a SWRI_PROFILE(getName()).  If the
    // nodelet is run as part of a nodelet manager, the node name and
    // nodelet name will differ and we want to append the node name to
    // allow us to guage the relative runtimes of all the instrumented
    // nodelets in that manager.  If the nodelet is run standalone,
    // then the nodelet name and node name will be the same and we
    // don't need to duplicate it.
    if (!stringStartsWith(label, node_name)) {
      label = node_name + "/" + label;
    }

    // Note that label is guaranteed to start with the ROS node's
    // name, without a leading slash.
    return trimSlash(label);
  }.bind(this);

  var keys = []
  var items = {}
  for (var i = 0; i < msg.data.length; i++) {
    var item = msg.data[i];
    var label = fullNameFromKey(item.key)
    if (label == undefined) {  
      continue;
    }

    items[label] = {
      label: label,
      abs_call_count: item.abs_call_count,
      abs_total_duration: toNSec(item.abs_total_duration),
      rel_total_duration: toNSec(item.rel_total_duration),
      rel_max_duration: toNSec(item.rel_max_duration)
    };
  } 
  this.data_handler(node_name, secs, items);
};

////////////////////////////////////////////////////////////////////////////////
// Profile Data Objects /////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////

// A BlockItem stores the raw data from a single measurement of a
// single block.
var BlockItem = function(src) {
  this.valid = false;
  this.abs_call_count =  0;
  this.rel_max_duration = 0;

  // Inclusive durations include time spent in children.
  this.inc_abs_duration = 0;
  this.inc_rel_duration = 0;
  // Exclusive durations do not include time spent in children.
  this.exc_abs_duration = 0;
  this.exc_rel_duration = 0;
};

var Block = function(size) {
  this.measured = false;
  this.data = new Array(size);
  for (var i = 0; i < size; i++) {
    this.data[i] = new BlockItem();
  }
}

// Find the last valid data that occurred before start_index.  Returns
// the corresponding index or -1 if no valid data was available.
Block.prototype.findPreviousValidData = function(start_index) {
  if (start_index == null) {
    start_index = this.data.length-1;
  }
  for (var i = start_index; i > -1; i--) {
    if (this.data[i].valid) {
      return i;
    }
  }
  return i;
}

// The GlobalProfile is the top level object containing all of the
// data and providing the functionality to extract various parts.
var GlobalProfile = function() {
  this.timeline = [];
  this.blocks = {"" : new Block(0)};
  this.tree_index = undefined;

  this.max_size = 300;
};

GlobalProfile.prototype.addData = function(node_name, t, items) {
  // Find the index in the timeline for this time.  If the time isn't
  // in the timeline, we need to add it and insert a corresponding
  // entry in every block's data array.
  var time_index = findTime(this.timeline, t)
  if (this.timeline[time_index] != t) {
    this.timeline.splice(time_index, 0, t);
    for (var block_name in this.blocks) {
      var block = this.blocks[block_name];
      block.data.splice(time_index, 0, new BlockItem());
    }
  }

  // Write the new items into the block data.  We touch each block to
  // make sure it and all of its parents exist in the blocks map.
  var rebuild_index = false;
  for (var block_name in items) {
    if (this.touchBlock(block_name)) {
      rebuild_index = true;
    }

    var block = this.blocks[block_name];
    block.measured = true;
    block.data[time_index].valid = true;
    block.data[time_index].abs_call_count = items[block_name].abs_call_count;
    block.data[time_index].inc_abs_duration = items[block_name].abs_total_duration;
    block.data[time_index].inc_rel_duration = items[block_name].rel_total_duration;
    block.data[time_index].rel_max_duration = items[block_name].rel_max_duration;
  }

  // If any new blocks were created (this should be rare), we rebuild
  // the tree index.
  if (rebuild_index) {
    this.rebuildIndex();
  }

  // Since we have new data, we need to update all of the estimated
  // and exclusive costs in the tree.  
  this.updateDerivedData(time_index);
 
  if (this.max_size > 0 && this.timeline.length > this.max_size) {
    this.timeline.shift();
    for (var block_name in this.blocks) {
      this.blocks[block_name].data.shift();
    }
  }      
};

GlobalProfile.prototype.touchBlock = function(name) {
  if (name in this.blocks) {
    return false;
  }
  
  var parts = name.split('/');
  for (var i = 0; i < parts.length; i++) {
    var block_name = parts.slice(0, i+1).join('/');
    if (!(block_name in this.blocks)) {
      this.blocks[block_name] = new Block(this.timeline.length);
    }
  }    
  return true;
};

var TreeItem = function(name, depth) {
  this.name = name;
  this.depth = depth;
  this.parent = undefined;
  this.children = [];
}

GlobalProfile.prototype.rebuildIndex = function() {
  // I like this approach because it lets us keep the children as
  // arrays without having to constantly search through the arrays to
  // find a node by name.  But maybe it will turn out that we can do
  // something easier.

  var keys = Object.keys(this.blocks);
  keys.sort();

  // We use this to compare lists that we know will be the same
  // length, and that are most likely to differ in the last elements.
  var equals = function(a,b) { 
    for (var i = a.length; i > 0; i--) {
      if (a[i-1] != b[i-1]) { return false; }
    }
    return true;
  };

  var joinParts = function(parts, size) {
    return parts.slice(0, size).join('/');
  }

  var stack = [];
  this.tree_index = new TreeItem('', 0);
  var current = this.tree_index;
  
  // Start at 1 because the first key is the root node.
  for (var i = 1; i < keys.length; i++) {
    // Split the key into its component parts.
    var parts = keys[i].split('/');

    // If the current stack doesn't equal the first part of this name,
    // we need to climb back down the tree.
    while (stack.length > 0 && !equals(stack, parts.slice(0, stack.length))) {
      stack.pop();
      current = current.parent;
    }

    while (stack.length != parts.length) {
      var name = joinParts(parts, stack.length+1);
      var new_item = new TreeItem(name, stack.length+1);
      new_item.parent = current;
      current.children.push(new_item);
      stack.push(parts[stack.length]);
      current = new_item;
    }
  }
};

GlobalProfile.prototype.updateDerivedData = function(index) { 
  var update = function(node) {
    var children_call_count = 0;
    var children_abs_duration = 0;
    var children_rel_duration = 0;

    for (var i = 0; i < node.children.length; i++) {
      update(node.children[i]);

      var child_block = this.blocks[node.children[i].name];
      // We have to do this because data comes in asynchronously.
      var child_index = child_block.findPreviousValidData(index);
      if (child_index >= 0) {
        var child_data = child_block.data[child_index];
        children_call_count += child_data.abs_call_count;
        children_abs_duration += child_data.inc_abs_duration;
        children_rel_duration += child_data.inc_rel_duration;
      }
    }

    var block = this.blocks[node.name];
    var data = this.blocks[node.name].data[index];
    if (!block.measured) {
      data.valid = true;
      // todo(exjohnson): consider adding abs_call_count and
      // rel_max_duration too.
      data.abs_call_count = children_call_count;
      data.inc_abs_duration = children_abs_duration;
      data.inc_rel_duration = children_rel_duration;
    }

    data.exc_abs_duration = Math.max(0, data.inc_abs_duration - children_abs_duration);
    data.exc_rel_duration = Math.max(0, data.inc_rel_duration - children_rel_duration);
  }.bind(this);

  update(this.tree_index);
}

////////////////////////////////////////////////////////////////////////////////
// Profile Data Object /////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////
var PartitionItem = function(name) {
  this.abs_name = name;
  this.rel_name = name.split('/').slice(-1)[0];
  this.block_item = null;
  this.value = 0;
  this.internal = false;
  this.children = [];
};

var partitionData = function(global_profile, index) {
  var build = function(node) {
    var block = global_profile.blocks[node.name];
    var valid_index = block.findPreviousValidData(index);
    if (valid_index < 0) { 
      return null; 
    }
    var item = new PartitionItem(node.name);
    item.block_item = block.data[valid_index];
    item.value = item.block_item.inc_abs_duration;    
        
    for (var i = 0; i < node.children.length; i++) {
      var child_item = build(node.children[i]);
      if (child_item) {
        item.children.push(child_item);
      }
    }

    if (item.children.length > 0 && item.block_item.exc_abs_duration > 0) {
      var internal_item = new PartitionItem(node.name + '/[Internal]');
      internal_item.block_item = item.block_item;
      internal_item.value = item.block_item.exc_abs_duration;
      internal_item.internal = true;
      item.children.unshift(internal_item);
    }
    
    return item;
  } 
  
  if (index == null) {
    index = global_profile.timeline.length-1;
  }
  return build(global_profile.tree_index);
}


var colorForName = function(name) {
  var rel_name = name.split('/').slice(-1)[0];
  var hash = stringHashCode(rel_name);
  return d3.rgb(hash & 0xFF, (hash >> 8) & 0xFF, (hash >> 16) & 0xFF);
};

var StackableItem = function(name) {
  this.abs_name = name;
  this.rel_name = name.split('/').slice(-1)[0];
  this.use_inclusive = false;
};


var stackingIndex = function(global_profile, namespace, depth) {
  var re = RegExp('^' + escapeRegExp(namespace));
  var names = Object.keys(global_profile.blocks).filter(
    function (s) { return re.test(s); }).sort();

  var depth0 = namespace.split('/').length;
  var depth1 = depth0 + depth;

  var data = [];
  for (var i = 0; i < names.length; i++) {
    var test_depth = names[i].split('/').length;
    if (names[i] != "") {
      test_depth += 1;
    }

    if (depth != null && test_depth > depth1) {
      continue;
    }

    var datum = new StackableItem(names[i]);
    if (depth != null && test_depth == depth1) {
      // If this item is the deepest that we are including, we should
      // use the inclusive cost instead of the exclusive cost to catch
      // the time spent in undisplayed children.  If the node doesn't
      // have children, the inclusive cost is equal to the exclusive
      // cost anyways, so it doesn't matter which we choose.
      datum.use_inclusive = true;
    }
   
    data.push(datum);
  }

  return data;
}


var stream_offset = 'silhouette';
var stream_interpolation = 'basis';
var stream_depth = null;
var stream_root = "";

var drawStreamgraph = function(global_profile) {  
  var stream_index = stackingIndex(global_profile, stream_root, stream_depth);
  var min_time = null;
  var max_time = null;


  var all_data = [];
  for (var i = 0; i < stream_index.length; i++) {
    var abs_name = stream_index[i].abs_name;
    var src_data = global_profile.blocks[abs_name].data;

    var values = new Array(src_data.length);
    
    for (var j = 0; j < src_data.length; j++) {
      var t = global_profile.timeline[j]
      var v = stream_index[i].use_inclusive ? src_data[j].inc_rel_duration : src_data[j].exc_rel_duration;
      values[j] = [t, v];
      min_time = nullMin(min_time, t);
      max_time = nullMax(max_time, t);
      // max_duration = max_duration == null ? v : Math.max(max_duration, v);
    }

    all_data.push({ "name": abs_name, "values": values});
  }

  var layout = d3.layout.stack()
    .offset(stream_offset)
    .values(function (layer) { return layer.values; })
    .x(function (d) { return d[0]; })
    .y(function (d) { return d[1]; });

  all_data = layout(all_data);

  var min_y = null;
  var max_y = null;
  var bot_data = all_data[0].values;
  var top_data = all_data.slice(-1)[0].values;
  for (var i = 0; i < src_data.length; i++) {
    max_y = Math.max(max_y, top_data[i].y+top_data[i].y0);
    min_y = Math.min(min_y, bot_data[i].y);
  }

  var x_scale = d3.scale.linear()
    .domain([min_time, max_time])
    .range([0, svg2.attr("width")]);
  
  var y_scale = d3.scale.linear()
    .domain([min_y, max_y])
    .range([0, svg2.attr("height")])

  var area = d3.svg.area()
    .x(function(d) { return x_scale(d[0]); })
    .y0(function(d) { return y_scale(d.y0); })
    .y1(function(d) { return y_scale(d.y0 + d.y); })
    .interpolate(stream_interpolation);

  var sel = svg2.selectAll("path")
    .data(all_data, function (d) { return d.name; });

  sel.attr("d", function (d) { return area(d.values); })    

  sel.enter().append("path")
    .attr("d", function (d) { return area(d.values); })    
    .style("fill", function(d) { return colorForName(d.name); })
    .style("fill-opacity", 0.6)
    .on("mouseover", function(d) { showTooltip(d.name); })
    .on("mouseout", function(d) { hideTooltip(); })
    .transition(200)
    .style("fill-opacity", 1);

  sel.exit()
    .transition(200)
    .style("fill-opacity", 0)
    .remove();

}

var showTooltip = function(name) {
  var tooltip = d3.select("#tooltip");
  tooltip.style("left", d3.event.pageX + "px")
    .style("top", d3.event.pageY + "px");
  tooltip.select("#label").text("/" + name);

  var block_data = profile_data.blocks[name];
  var i = block_data.findPreviousValidData(null);

  var d = new BlockItem();
  if (i >= 0) {
    d = block_data.data[i];
  }

  tooltip.select("#calls").text(d.valid ? d.abs_call_count : "N/A");
  tooltip.select("#inc_total_duration").text(formatDuration(d.inc_abs_duration));  
  tooltip.select("#exc_total_duration").text(formatDuration(d.exc_abs_duration));  

  tooltip.select("#inc_avg_duration").text(
    d.abs_call_count ? formatDuration(d.inc_abs_duration / d.abs_call_count) : "N/A");  
  tooltip.select("#exc_avg_duration").text(
    d.abs_call_count ? formatDuration(d.exc_abs_duration / d.abs_call_count) : "N/A");  

  tooltip.select("#inc_instant_duration").text(formatDuration(d.inc_rel_duration));  
  tooltip.select("#exc_instant_duration").text(formatDuration(d.exc_rel_duration));  

  // tooltip.select("#avg_duration").text(
  // d.measured && d.call_count > 0 ? formatDuration(d.total_duration / d.call_count) : "N/A");  
  // tooltip.select("#max_duration").text(d.measured ? formatDuration(d.abs_max_duration) : "N/A");

  d3.select("#tooltip").classed("hidden", false);
}

var hideTooltip = function() {
  d3.select("#tooltip").classed("hidden", true);
}

var drawCanvasRect = function(global_profile) {
  var partition = d3.layout.partition()
    .sort(null);

  var x_scale = d3.scale.linear()
    .domain([0.0, 1.0])
    .range([0, svg1.attr("width")]);
  
  var y_scale = d3.scale.linear()
    .domain([0.0,1.0])
    .range([0, svg1.attr("height")]);

  var data = partitionData(global_profile);

  var rects = svg1.selectAll("rect")
    .data(partition.nodes(data), function (d) { return d.abs_name; });

  rects.enter()
    .append("rect")
    .attr("display", function(d) { return d.internal ? "none" : null; })
    .attr("x", function(d) { return x_scale(d.y); })
    .attr("y", function(d) { return y_scale(d.x); })
    .attr("width", function(d) { return x_scale(d.y+d.dy) - x_scale(d.y); })
    .attr("height", function(d) { return 0; })
    .style("stroke", "#fff")
    .attr("fill", function (d) { return colorForName(d.rel_name); })
    .on("mouseover", function (d) { showTooltip(d.abs_name); })
    .on("mouseout", hideTooltip)

  rects
    .transition()
    .duration(200)
    .attr("x", function(d) { return x_scale(d.y); })
    .attr("y", function(d) { return y_scale(d.x); })
    .attr("width", function(d) { return x_scale(d.y+d.dy) - x_scale(d.y); })
    .attr("height", function(d) { return Math.abs(y_scale(d.x) - y_scale(d.x+d.dx)); })
          
};

var svg1 = d3.select("#partition_svg");
var svg2 = d3.select("#stream_svg");

var update_timer = null;
var handleUpdateTimer = function() {
  update_timer = null;
  if (profile_data.timeline.length > 0) {
    drawCanvasRect(profile_data);
    drawStreamgraph(profile_data);
  }
}

var update = function() {
  if (update_timer) { window.clearTimeout(update_timer); }
  update_timer = window.setTimeout(handleUpdateTimer, 50);
}

// if (!true) {
//   updateCanvasRect();
// } else {
//   updateCanvasArc();
// }

var ros = new ROSLIB.Ros({url : 'ws://localhost:9091'});
var ros_adapter = undefined;
var profile_data = new GlobalProfile();

ros.on('connection', function() { 
  console.log('Connected to websocket server.'); 
  ros_adapter = new RosProfilerAdapter(ros); 
  ros_adapter.dataHandler(function(n,t,m) {
    profile_data.addData(n,t,m);
    update();
  });
});
ros.on('error', function(error) { console.log('Error connecting to websocket server: ', error); });
ros.on('close', function() { 
  console.log('Connection to websocket server closed.'); 
  ros_adapter.close();
});      


var partition = d3.layout.partition()
//.sort(null)
  .sort(function(a,b) { 
    if (a.internal) { return -1; } 
    else if (b.internal) { return 1; }
    else return d3.ascending(a.abs_name, b.abs_name); })
  .value(function (d) { return d.total_duration; });



$('input[type=radio][name=offset]').change(function() {
  stream_offset = this.id;
  update();
});

$('input[type=radio][name=interpolation]').change(function() {
  stream_interpolation = this.id;
  update();
});



var depth_spinner = $("#depth_spinner").spinner()
  .spinner("value", 10)
  .spinner("option", "min", 0)
  .spinner("option", "max", 100)
  .on("spin", function (event, ui) {
    stream_depth=ui.value;
    update();
  })
  .on("spinchange", function(event, ui) {
    stream_depth=depth_spinner.spinner("value");
    update();
  });
