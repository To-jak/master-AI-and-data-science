// Map Dimensions
const w_map_svg = 640;
const h_map_svg = 640;
const map_margin = { top: 30, right: 20, bottom: 50, left: 60 };

const w_map = w_map_svg - map_margin.left - map_margin.right
const h_map = h_map_svg - map_margin.top - map_margin.bottom

// Elements for drawing map
let x;
let y;

const low_density_color = "blue"
const high_density_color = "red"
let colorScale;

// Dynamic axis
let xAxis;
let yAxis;

// Histogram Dimensions
const w_hist_svg = 400;
const h_hist_svg = 350;
const hist_margin = { top: 10, right: 40, bottom: 40, left: 40 };

const w_hist = w_hist_svg - hist_margin.left - hist_margin.right
const h_hist = h_hist_svg - hist_margin.top - hist_margin.bottom

// Elements for drawing histogram
let x_hist;
let y_hist;
let histogram;
let num_bins = 10;
let last_bin_limit = 6000;

let dataset = [];

// ZOOM method
const zoom = d3.zoom().scaleExtent([1, 8])
    .on("zoom", function () {

        map_content.attr("transform", d3.event.transform)

        // Recover new scale
        var newX = d3.event.transform.rescaleX(x);
        var newY = d3.event.transform.rescaleY(y);

        // Update axes
        xAxis.call(d3.axisBottom(newX))
        yAxis.call(d3.axisLeft(newY))
    });

// Map SVG element definition
let svg = d3.select("#map").append("svg").attr("class", "svg")
    .attr("width", w_map_svg)
    .attr("height", h_map_svg)
    .call(zoom);

// Map Background
svg.append("rect")
    .attr("width", w_map)
    .attr("height", h_map)
    .attr("x", map_margin.left)
    .attr("y", map_margin.top)
    .attr("fill", "white")
    .attr("opacity", 0.7)
    .attr("stroke", "black");


// Append a defs element to Map SVG
var defs = svg.append('defs');

// Clip area for Map map
let clip = defs.append("clipPath")
    .attr("id", "clip")
    .append("rect")
    .attr("width", w_map)
    .attr("height", h_map)
    .attr("x", map_margin.left)
    .attr("y", map_margin.top);

// LINEAR GRADIENT for legend
var linearGradient = defs.append("linearGradient")
    .attr("id", "linear-gradient")
    //Horizontal gradient
    .attr("x1", "0%")
    .attr("y1", "0%")
    .attr("x2", "100%")
    .attr("y2", "0%");

//Set the color for the start (0%)
linearGradient.append("stop")
    .attr("offset", "0%")
    .attr("stop-color", low_density_color);

//Set the color for the end (100%)
linearGradient.append("stop")
    .attr("offset", "100%")
    .attr("stop-color", high_density_color);

// Map content area
let map_content = svg.append("g").attr("clip-path", "url(#clip)")
    .append("g")
    .attr("transform", "translate(" + map_margin.left + "," + map_margin.top + ")");

// Histogram content area
let histogram_content = d3.select("#histogram").append("svg").attr("class", "svg")
    .attr("width", w_hist_svg)
    .attr("height", h_hist_svg)
    .append("g")
    .attr("transform", "translate(" + hist_margin.left + "," + hist_margin.top + ")");

// LOAD DATA THEN DRAW
d3.tsv("data/france.tsv")
    .row((d, i) => {
        return {
            codePostal: +d["Postal Code"],
            inseeCode: +d.inseecode,
            place: d.place,
            longitude: +d.x,
            latitude: +d.y,
            population: +d.population,
            density: +d.density
        };
    })
    .get((error, rows) => {
        console.log("Loaded " + rows.length + " rows");

        if (rows.length > 0) {
            console.log("First row: ", rows[0])
            console.log("Last row: ", rows[rows.length - 1])

            dataset = rows;

            x = d3.scaleLinear()
                .domain(d3.extent(rows, (row) => row.longitude))
                .range([0, w_map]);

            x_hist = d3.scaleLinear()
                .domain([0, last_bin_limit])
                .range([0, w_hist]);

            y = d3.scaleLinear()
                .domain(d3.extent(rows, (row) => row.latitude))
                .range([h_map, 0]);

            colorScale = d3.scaleLinear()
                .domain(d3.extent(rows, (row) => row.density))
                .range([low_density_color, high_density_color]);

            // Histogram definition
            histogram = d3.histogram()
                .value(function (d) { return d.population > last_bin_limit ? last_bin_limit : d.population; })
                .domain(x_hist.domain())
                .thresholds(num_bins);

            draw_map();
            draw_histogram();
        }
    }
    );

// Draw Map
function draw_map() {

    map_content.selectAll("circle")
        .data(dataset)
        .enter()
        .append("circle")
        .attr("width", 1)
        .attr("height", 1)
        .attr("r", 0.8)
        .attr("cx", (d) => x(d.longitude))
        .attr("cy", (d) => y(d.latitude))
        .attr("fill", function (d) { return colorScale(d.population) })
        .on('mouseover', function (d, i) {
            tooltip.style("visibility", "visible").style("top", (event.pageY - 10) + "px").style("left", (event.pageX + 10) + "px")
                .text(d.place);
        })
        .on("mouseout", function () {
            return tooltip.style("visibility", "hidden")
        })
        .on("click", function (d, i) {
            d3.select("#text-info").html(generate_text_info(d))
        });

    // Draw x axis
    xAxis = svg.append("g")
        .attr("class", "x_axis")
        .attr("transform", "translate(" + map_margin.left + ", "
            + (h_map_svg - map_margin.bottom) + ")")
        .call(d3.axisBottom(x));

    svg.append("text")
        .style("text-anchor", "middle")
        .attr("x", w_map_svg / 2)
        .attr("y", (map_margin.top + h_map + 34))
        .style("font-size", "12px")
        .text("Longitude")

    // Draw y axis
    yAxis = svg.append("g")
        .attr("class", "y_axis")
        .attr("transform", "translate(" + map_margin.left + ", " + map_margin.top + ")")
        .call(d3.axisLeft(y));

    svg.append("text")
        .style("text-anchor", "middle")
        .attr("transform", "rotate(-90)")
        .attr("x", -h_map_svg / 2)
        .attr("y", map_margin.left - 34)
        .style("font-size", "12px")
        .text("Latitude")

    // Draw legend rectangle and fill with gradient
    svg.append("rect")
        .attr("x", 10)
        .attr("y", 10)
        .attr("width", 100)
        .attr("height", 10)
        .style("fill", "url(#linear-gradient)");

    // Append legend title
    svg.append("text")
        .attr("class", "legend")
        .attr("x", 120)
        .attr("y", 20)
        .style("text-anchor", "left")
        .style("font-size", "12px")
        .text("Population Density");
}

function generate_text_info(data) {
    return "City: <b>" + data.place + "</b>, <b>" + data.codePostal
        + "</b><br> Density: <b>" + data.density.toLocaleString() + "</b> inhabitants per km²"
        + "<br> Population: <b>" + data.population.toLocaleString() + "</b> inhabitants"
}

// Draw Histogram
function draw_histogram() {

    histogram_content.append("g")
        .attr("transform", "translate(0," + h_hist + ")")
        .call(d3.axisBottom(x_hist))

    histogram_content.append("text")
        .attr("transform", "translate(" + (w_hist + 14) + ", " + (h_hist + 16) + ")")
        .style("font-size", "13px")
        .text("+");

    histogram_content.append("text")
        .style("text-anchor", "middle")
        .attr("x", w_hist / 2)
        .attr("y", (h_hist + hist_margin.bottom - 5))
        .style("font-size", "12px")
        .text("Population Histogram")

    // Get the histogram bins
    var bins = histogram(dataset);

    console.log(bins);

    // Y axis
    y_hist = d3.scaleLinear()
        .range([h_hist, 0]);

    y_hist.domain([0, d3.max(bins, function (d) { return d.length; })]);
    histogram_content.append("g")
        .call(d3.axisLeft(y_hist));

    // Append the bar rectangles to the histogram svg element
    histogram_content.selectAll("rect")
        .data(bins)
        .enter()
        .append("rect")
        .attr("x", 1)
        .attr("id", function (d, i) { return "bar" + i })
        .attr("selected", false)
        .attr("transform", function (d) { return "translate(" + x_hist(d.x0) + "," + y_hist(d.length) + ")"; })
        .attr("width", function (d) { return x_hist(d.x1) - x_hist(d.x0) - 1; })
        .attr("height", function (d) { return h_hist - y_hist(d.length); })
        .style("fill", "blue")
        .on('click', function (d, i) {
            histogramClick(d, i)
        })
}

function histogramClick(d, i) {

    currentBar = histogram_content.select("rect[id=bar" + i + "]")
    already_selected = currentBar.attr("selected") == "true"

    d3.select("#map").selectAll("circle").attr("visibility", "visible");
    histogram_content.selectAll("rect").attr("selected", false).style("fill", "blue");

    if (already_selected) {
        return;
    }

    currentBar.attr("selected", true).style("fill", "red")

    var lower = d.x0
    var higher = d.x1

    if (higher == last_bin_limit) {
        d3.select("#map").selectAll("circle").filter(function (d) {
            return d.population <= lower
        }).attr("visibility", "hidden")
    }
    else {
        d3.select("#map").selectAll("circle").filter(function (d) {
            return d.population <= lower || higher <= d.population
        }).attr("visibility", "hidden")
    }
}

// TOOLTIP
var tooltip = d3.select("body")
    .append("div")
    .style("position", "absolute")
    .style("z-index", "10")
    .style("visibility", "hidden")
    .style("padding", "5px")
    .style("background-color", "black")
    .style("color", "white")
    .style("border-radius", "5px")
    .text("");