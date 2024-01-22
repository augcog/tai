

TurtleBot3

 {
 "@context" : "http://schema.org",
 "@type" : "Person",
 "name" : "ROBOTIS",
 "url" : "https://emanual.robotis.com",
 "sameAs" : null
 }

$(function() {
 return $("h1, h2, h3, h4, h5, h6").each(function(i, el) {
 var $el, icon, id;
 $el = $(el);
 id = $el.attr('id');
 icon = '<i class="fa fa-link"></i>';
 if (id) {
 return $el.append($("<a />").addClass("header-link").attr("href", "#" + decodeURI(id)).attr("onclick", "copyurl(this);").html(icon));
 }
 });
});

$(function(){
 $('h1').each(function(){
 $(this).addClass('class-h1');
 });
 $('h2').each(function(){
 $(this).addClass('class-h2');
 });
 $('h3').each(function(){
 $(this).addClass('class-h3');
 });
 $('h4').each(function(){
 $(this).addClass('class-h4');
 });
 $('h5').each(function(){
 $(this).addClass('class-h5');
 });
 $('h6').each(function(){
 $(this).addClass('class-h6');
 });
});

$(function() {
 $('h1, h2, h3, h4, h5, h6').each(function(i, e) {
 $(e).nextUntil('h1, h2, h3, h4, h5, h6').wrapAll('<div>');
 });

});

 var toggle\_google = 0;
 var toggle\_search\_tool = 0;
 var toggle\_search\_result = 0;
 var mobile\_width = 960;

 function extendgoogletool() {
 if (toggle\_google == 0) {
 google\_tools.classList.add("extend\_googletools");
 switch\_ko\_en.classList.add("show\_switch\_ko\_en");
 toggle\_google = 1;
 }
 else {
 google\_tools.classList.remove("extend\_googletools");
 switch\_ko\_en.classList.remove("show\_switch\_ko\_en");
 toggle\_google = 0;
 }
 }

 function extendsearchtool() {
 if (toggle\_search\_tool == 0) {
 $('.searchbox').css('display', 'block');
 $('#archive').css('margin-top', '40px');
 toggle\_search\_tool = 1;
 }
 else {
 $('.searchbox').css('display', 'none');
 $('#archive').css('margin-top', '0px');
 toggle\_search\_tool = 0;
 }
 }

 $(document).ready(function() {
 $('.blank').attr('target', '\_blank');
 $('.popup').magnificPopup({
 type: 'ajax',
 alignTop: true,
 overflowY: 'scroll' // as we know that popup content is tall we set scroll overflow by default to avoid jump
 });
 $('.popup2').magnificPopup({
 type: 'inline',
 alignTop: true,
 overflowY: 'scroll' // as we know that popup content is tall we set scroll overflow by default to avoid jump
 });
 });

 $(function () {
 var currentHash = $(location).attr('hash');
 var filename = $(location).attr('href').split('/').reverse()[1];
 $(document).scroll(function () {
 $('.header-link').each(function () {
 var top = window.pageYOffset;
 var distance = top - $(this).offset().top;
 var rawHash = $(this).attr('href');
 var contentId = rawHash.replace('#', '');
 var tocId = rawHash.replace('#', filename+'\_toc\_');
 var searchContentId = $('#'+contentId);
 var searchTocId = $('#'+tocId);

 if (distance < 50 && distance > -50 && currentHash != tocId) {
 // window.history.pushState(window.location.href, "title", currentHash); //Update browser URL history with current anchor
 if ((searchContentId.hasClass('class-h1') || searchContentId.hasClass('class-h2') || searchContentId.hasClass('class-h3')) && $('.nav\_\_items').has('#'+tocId).length) {
 // console.log(contentId);
 // console.log(tocId);
 searchTocId.prop("checked", true); //Open folded Toc menu
 $('.current\_contents').removeClass("current\_contents");

 //Check if Buttoned ToC with children
 // console.log(searchTocId);
 if( searchTocId.prop("tagName") == "INPUT" ) {
 // console.log("input detected");
 if( searchTocId.prop("name") == "sublevel" ) {
 searchTocId.closest("li.has-grandchildren").find("label > button").addClass("current\_contents");
 }
 }
 else {
 searchTocId.addClass("current\_contents");
 }

 // Open folded Toc menu
 searchTocId.closest("li.has-grandchildren").find("input").first().prop("checked", true);
 searchTocId.closest("li.has-children").find("input").first().prop("checked", true);

 currentHash = tocId;
 }
 }
 });
 });
 });

 window.onscroll = function () {
 var offset = 64 - $(window).scrollTop();
 if(offset < 0) { offset = 0; }
 $(".sidebar\_\_left").css("top", offset); //sticky sidebar
 $(".extend\_sidebar").css("top", offset);
 $(".local\_navigation").css("top", offset); //sticky local navigation
 $(".searchbox").css("top", offset); //sticky searchbox
 // console.log(offset);
 };

 var toggle = 0;

 function extendnav() {
 if (toggle == 0) {
 sidebar\_\_left.classList.add("extend\_sidebar");
 toggle = 1;
 }
 else {
 sidebar\_\_left.classList.remove("extend\_sidebar");
 toggle = 0;
 }
 }

 function copyurl(item)
 {
 // location.href = item.href;
 var copiedurl = decodeURI(item.href);
 var tempurl = $('<input>').val(copiedurl).appendTo('body').select();
 document.execCommand('copy');
 // console.log(copiedurl);
 // document.getElementById(id).href;
 // document.execCommand('copy');
 }

 function movetopage(targeturl)
 {
 location.href=targeturl;
 }

 function tab\_handler(en){
 console.log(en.textContent);
 var tabs = document.getElementById('tabs').children;
 for(i = 0; i < tabs.length; i++) {
 tabs[i].className = "tab not\_\_selected";
 $(".archive").removeClass(tabs[i].textContent);
 }
 en.className = "tab selected";
 $(".archive").addClass(en.textContent);
 }

 // When a page is reloaded, a previously selected tab and tab\_contents will be recalled to the current page keeping the same status until user's windows is closed.
 $(document).ready(function() {
 //defualt Tab Status. 
 if(sessionStorage.getItem("tabActive") == null){
 console.log("tabActive:" + sessionStorage.getItem("tabActive")); 
 $(".tab:nth-child(1)").addClass("selected");
 $(".tab\_contents:nth-of-type(1)").addClass("active");
 } else if(sessionStorage.getItem("tabActive") !== null) {
 console.log("tabActive: Not Null"); 
 $('[data-id="' + sessionStorage.getItem("tabActive")+'"]').addClass("active");
 $('[data-tab-name="'+ sessionStorage.getItem("tabActive")+'"]').addClass("tab selected");
 }
 })

 // en = enable
 function tab\_handler(en){
 // console.log(en.textContent);
 var tabs = document.getElementById('tabs').children;
 for(i = 0; i < tabs.length; i++) {
 tabs[i].className = "tab not\_\_selected";
 // $(".archive").removeClass(tabs[i].textContent);
 }
 en.className = "tab selected";
 // $(".archive").addClass(en.textContent);
 show\_contents(en.textContent);
 }

 // Controling a clicked tab's contents when a click event occurs. 
 function show\_contents(temp){
 // Seaching active class within .tab\_contents classes in the DOM tree, and remove the "active" class and set the disply as none. 
 $(".tab\_contents.active").removeClass("active");
 // in the "tab\_contents" class, searching for a matched ID to add a new class to the speidic ID. 
 $(".tab\_contents").each(function(){
 // Searching for a ID parameter "temp" and add a class, active, and css style. 
 // $("#" + temp).addClass("active");
 $('[data-id="' + temp +'"]').addClass("active");
 console.log(temp+":"+ $('.tab\_contents').hasClass("active")); 
 // To store user's data in the local storage, collect a class data of the active class.
 sessionStorage.setItem('tabActive', temp); 
 })
 }

![](/assets/images/search_icon.png)

[![](/assets/images/robotis_emanual_logo.png)](http://emanual.robotis.com "http://emanual.robotis.com")

* DYNAMIXEL
	+ P Series
		- [PH54-200-S500-R (H54P)](https://emanual.robotis.com/docs/en/dxl/p/ph54-200-s500-r/ "https://emanual.robotis.com/docs/en/dxl/p/ph54-200-s500-r/")
		- [PH54-100-S500-R (H54P)](https://emanual.robotis.com/docs/en/dxl/p/ph54-100-s500-r/ "https://emanual.robotis.com/docs/en/dxl/p/ph54-100-s500-r/")
		- [PH42-020-S300-R (H42P)](https://emanual.robotis.com/docs/en/dxl/p/ph42-020-s300-r/ "https://emanual.robotis.com/docs/en/dxl/p/ph42-020-s300-r/")
		- [PM54-060-S250-R (M54P)](https://emanual.robotis.com/docs/en/dxl/p/pm54-060-s250-r/ "https://emanual.robotis.com/docs/en/dxl/p/pm54-060-s250-r/")
		- [PM54-040-S250-R (M54P)](https://emanual.robotis.com/docs/en/dxl/p/pm54-040-s250-r/ "https://emanual.robotis.com/docs/en/dxl/p/pm54-040-s250-r/")
		- [PM42-010-S260-R (M42P)](https://emanual.robotis.com/docs/en/dxl/p/pm42-010-s260-r/ "https://emanual.robotis.com/docs/en/dxl/p/pm42-010-s260-r/")
	+ DYNAMIXEL DRIVE (DYD)
		- [DYD-11](https://emanual.robotis.com/docs/en/all-dyd/dyd-11 "https://emanual.robotis.com/docs/en/all-dyd/dyd-11")
		- [DYD-14](https://emanual.robotis.com/docs/en/all-dyd/dyd-14 "https://emanual.robotis.com/docs/en/all-dyd/dyd-14")
		- [DYD-17](https://emanual.robotis.com/docs/en/all-dyd/dyd-17 "https://emanual.robotis.com/docs/en/all-dyd/dyd-17")
	+ [X Series](https://emanual.robotis.com/docs/en/dxl/x/ "https://emanual.robotis.com/docs/en/dxl/x/")
	+ XW Series
		- [XW540-T140](https://emanual.robotis.com/docs/en/dxl/x/xw540-t140/ "https://emanual.robotis.com/docs/en/dxl/x/xw540-t140/")
		- [XW540-T260](https://emanual.robotis.com/docs/en/dxl/x/xw540-t260/ "https://emanual.robotis.com/docs/en/dxl/x/xw540-t260/")
		- [XW430-T200](https://emanual.robotis.com/docs/en/dxl/x/xw430-t200/ "https://emanual.robotis.com/docs/en/dxl/x/xw430-t200/")
		- [XW430-T333](https://emanual.robotis.com/docs/en/dxl/x/xw430-t333/ "https://emanual.robotis.com/docs/en/dxl/x/xw430-t333/")
	+ XD Series
		- [XD540-T150](https://emanual.robotis.com/docs/en/dxl/x/xd540-t150/ "https://emanual.robotis.com/docs/en/dxl/x/xd540-t150/")
		- [XD540-T270](https://emanual.robotis.com/docs/en/dxl/x/xd540-t270/ "https://emanual.robotis.com/docs/en/dxl/x/xd540-t270/")
		- [XD430-T210](https://emanual.robotis.com/docs/en/dxl/x/xd430-t210/ "https://emanual.robotis.com/docs/en/dxl/x/xd430-t210/")
		- [XD430-T350](https://emanual.robotis.com/docs/en/dxl/x/xd430-t350/ "https://emanual.robotis.com/docs/en/dxl/x/xd430-t350/")
	+ XH Series
		- [XH540-W150](https://emanual.robotis.com/docs/en/dxl/x/xh540-w150/ "https://emanual.robotis.com/docs/en/dxl/x/xh540-w150/")
		- [XH540-W270](https://emanual.robotis.com/docs/en/dxl/x/xh540-w270/ "https://emanual.robotis.com/docs/en/dxl/x/xh540-w270/")
		- [XH540-V150](https://emanual.robotis.com/docs/en/dxl/x/xh540-v150/ "https://emanual.robotis.com/docs/en/dxl/x/xh540-v150/")
		- [XH540-V270](https://emanual.robotis.com/docs/en/dxl/x/xh540-v270/ "https://emanual.robotis.com/docs/en/dxl/x/xh540-v270/")
		- [XH430-W210](https://emanual.robotis.com/docs/en/dxl/x/xh430-w210/ "https://emanual.robotis.com/docs/en/dxl/x/xh430-w210/")
		- [XH430-W350](https://emanual.robotis.com/docs/en/dxl/x/xh430-w350/ "https://emanual.robotis.com/docs/en/dxl/x/xh430-w350/")
		- [XH430-V210](https://emanual.robotis.com/docs/en/dxl/x/xh430-v210/ "https://emanual.robotis.com/docs/en/dxl/x/xh430-v210/")
		- [XH430-V350](https://emanual.robotis.com/docs/en/dxl/x/xh430-v350/ "https://emanual.robotis.com/docs/en/dxl/x/xh430-v350/")
	+ XM Series
		- [XM540-W150](https://emanual.robotis.com/docs/en/dxl/x/xm540-w150/ "https://emanual.robotis.com/docs/en/dxl/x/xm540-w150/")
		- [XM540-W270](https://emanual.robotis.com/docs/en/dxl/x/xm540-w270/ "https://emanual.robotis.com/docs/en/dxl/x/xm540-w270/")
		- [XM430-W210](https://emanual.robotis.com/docs/en/dxl/x/xm430-w210/ "https://emanual.robotis.com/docs/en/dxl/x/xm430-w210/")
		- [XM430-W350](https://emanual.robotis.com/docs/en/dxl/x/xm430-w350/ "https://emanual.robotis.com/docs/en/dxl/x/xm430-w350/")
	+ XC Series
		- [2XC430-W250](https://emanual.robotis.com/docs/en/dxl/x/2xc430-w250/ "https://emanual.robotis.com/docs/en/dxl/x/2xc430-w250/")
		- [XC430-W150](https://emanual.robotis.com/docs/en/dxl/x/xc430-w150/ "https://emanual.robotis.com/docs/en/dxl/x/xc430-w150/")
		- [XC430-W240](https://emanual.robotis.com/docs/en/dxl/x/xc430-w240/ "https://emanual.robotis.com/docs/en/dxl/x/xc430-w240/")
		- [XC430-T150BB](https://emanual.robotis.com/docs/en/dxl/x/xc430-t150bb/ "https://emanual.robotis.com/docs/en/dxl/x/xc430-t150bb/")
		- [XC430-T240BB](https://emanual.robotis.com/docs/en/dxl/x/xc430-t240bb/ "https://emanual.robotis.com/docs/en/dxl/x/xc430-t240bb/")
		- [XC330-T288](https://emanual.robotis.com/docs/en/dxl/x/xc330-t288/ "https://emanual.robotis.com/docs/en/dxl/x/xc330-t288/")
		- [XC330-T181](https://emanual.robotis.com/docs/en/dxl/x/xc330-t181/ "https://emanual.robotis.com/docs/en/dxl/x/xc330-t181/")
		- [XC330-M181](https://emanual.robotis.com/docs/en/dxl/x/xc330-m181/ "https://emanual.robotis.com/docs/en/dxl/x/xc330-m181/")
		- [XC330-M288](https://emanual.robotis.com/docs/en/dxl/x/xc330-m288/ "https://emanual.robotis.com/docs/en/dxl/x/xc330-m288/")
	+ XL Series
		- [2XL430-W250](https://emanual.robotis.com/docs/en/dxl/x/2xl430-w250/ "https://emanual.robotis.com/docs/en/dxl/x/2xl430-w250/")
		- [XL430-W250](https://emanual.robotis.com/docs/en/dxl/x/xl430-w250/ "https://emanual.robotis.com/docs/en/dxl/x/xl430-w250/")
		- [XL330-M077](https://emanual.robotis.com/docs/en/dxl/x/xl330-m077/ "https://emanual.robotis.com/docs/en/dxl/x/xl330-m077/")
		- [XL330-M288](https://emanual.robotis.com/docs/en/dxl/x/xl330-m288/ "https://emanual.robotis.com/docs/en/dxl/x/xl330-m288/")
		- [XL-320](https://emanual.robotis.com/docs/en/dxl/x/xl320/ "https://emanual.robotis.com/docs/en/dxl/x/xl320/")
	+ MX Series
		- [MX-106T/R(2.0)](https://emanual.robotis.com/docs/en/dxl/mx/mx-106-2/ "https://emanual.robotis.com/docs/en/dxl/mx/mx-106-2/")
		- [MX-64T/R/AT/AR(2.0)](https://emanual.robotis.com/docs/en/dxl/mx/mx-64-2/ "https://emanual.robotis.com/docs/en/dxl/mx/mx-64-2/")
		- [MX-28T/R/AT/AR(2.0)](https://emanual.robotis.com/docs/en/dxl/mx/mx-28-2/ "https://emanual.robotis.com/docs/en/dxl/mx/mx-28-2/")
		- [MX-106T/R](https://emanual.robotis.com/docs/en/dxl/mx/mx-106/ "https://emanual.robotis.com/docs/en/dxl/mx/mx-106/")
		- [MX-64T/R/AT/AR](https://emanual.robotis.com/docs/en/dxl/mx/mx-64/ "https://emanual.robotis.com/docs/en/dxl/mx/mx-64/")
		- [MX-28T/R/AT/AR](https://emanual.robotis.com/docs/en/dxl/mx/mx-28/ "https://emanual.robotis.com/docs/en/dxl/mx/mx-28/")
		- [MX-12W](https://emanual.robotis.com/docs/en/dxl/mx/mx-12w/ "https://emanual.robotis.com/docs/en/dxl/mx/mx-12w/")
	+ AX Series
		- [AX-18F/18A](https://emanual.robotis.com/docs/en/dxl/ax/ax-18a/ "https://emanual.robotis.com/docs/en/dxl/ax/ax-18a/")
		- [AX-12/12+/12A](https://emanual.robotis.com/docs/en/dxl/ax/ax-12a/ "https://emanual.robotis.com/docs/en/dxl/ax/ax-12a/")
		- [AX-12W](https://emanual.robotis.com/docs/en/dxl/ax/ax-12w/ "https://emanual.robotis.com/docs/en/dxl/ax/ax-12w/")
	+ [DYNAMIXEL Protocol 1.0](https://emanual.robotis.com/docs/en/dxl/protocol1/ "https://emanual.robotis.com/docs/en/dxl/protocol1/")
	+ [DYNAMIXEL Protocol 2.0](https://emanual.robotis.com/docs/en/dxl/protocol2/ "https://emanual.robotis.com/docs/en/dxl/protocol2/")
	+ EX Series 
		- [EX-106+](https://emanual.robotis.com/docs/en/dxl/ex/ex-106+/ "https://emanual.robotis.com/docs/en/dxl/ex/ex-106+/")
	+ DX Series 
		- [DX-113](https://emanual.robotis.com/docs/en/dxl/dx/dx-113/ "https://emanual.robotis.com/docs/en/dxl/dx/dx-113/")
		- [DX-116](https://emanual.robotis.com/docs/en/dxl/dx/dx-116/ "https://emanual.robotis.com/docs/en/dxl/dx/dx-116/")
		- [DX-117](https://emanual.robotis.com/docs/en/dxl/dx/dx-117/ "https://emanual.robotis.com/docs/en/dxl/dx/dx-117/")
	+ RX Series 
		- [RX-10](https://emanual.robotis.com/docs/en/dxl/rx/rx-10/ "https://emanual.robotis.com/docs/en/dxl/rx/rx-10/")
		- [RX-24F](https://emanual.robotis.com/docs/en/dxl/rx/rx-24f/ "https://emanual.robotis.com/docs/en/dxl/rx/rx-24f/")
		- [RX-28](https://emanual.robotis.com/docs/en/dxl/rx/rx-28/ "https://emanual.robotis.com/docs/en/dxl/rx/rx-28/")
		- [RX-64](https://emanual.robotis.com/docs/en/dxl/rx/rx-64/ "https://emanual.robotis.com/docs/en/dxl/rx/rx-64/")
	+ PRO Series 
		- [H54-200-S500-R(A)](https://emanual.robotis.com/docs/en/dxl/pro/h54-200-s500-ra/ "https://emanual.robotis.com/docs/en/dxl/pro/h54-200-s500-ra/")
		- [H54-100-S500-R(A)](https://emanual.robotis.com/docs/en/dxl/pro/h54-100-s500-ra/ "https://emanual.robotis.com/docs/en/dxl/pro/h54-100-s500-ra/")
		- [H42-20-S300-R(A)](https://emanual.robotis.com/docs/en/dxl/pro/h42-20-s300-ra/ "https://emanual.robotis.com/docs/en/dxl/pro/h42-20-s300-ra/")
		- [M54-60-S250-R(A)](https://emanual.robotis.com/docs/en/dxl/pro/m54-60-s250-ra/ "https://emanual.robotis.com/docs/en/dxl/pro/m54-60-s250-ra/")
		- [M54-40-S250-R(A)](https://emanual.robotis.com/docs/en/dxl/pro/m54-40-s250-ra/ "https://emanual.robotis.com/docs/en/dxl/pro/m54-40-s250-ra/")
		- [M42-10-S260-R(A)](https://emanual.robotis.com/docs/en/dxl/pro/m42-10-s260-ra/ "https://emanual.robotis.com/docs/en/dxl/pro/m42-10-s260-ra/")
		- [H54-200-S500-R](https://emanual.robotis.com/docs/en/dxl/pro/h54-200-s500-r/ "https://emanual.robotis.com/docs/en/dxl/pro/h54-200-s500-r/")
		- [H54-100-S500-R](https://emanual.robotis.com/docs/en/dxl/pro/h54-100-s500-r/ "https://emanual.robotis.com/docs/en/dxl/pro/h54-100-s500-r/")
		- [H42-20-S300-R](https://emanual.robotis.com/docs/en/dxl/pro/h42-20-s300-r/ "https://emanual.robotis.com/docs/en/dxl/pro/h42-20-s300-r/")
		- [M54-60-S250-R](https://emanual.robotis.com/docs/en/dxl/pro/m54-60-s250-r/ "https://emanual.robotis.com/docs/en/dxl/pro/m54-60-s250-r/")
		- [M54-40-S250-R](https://emanual.robotis.com/docs/en/dxl/pro/m54-40-s250-r/ "https://emanual.robotis.com/docs/en/dxl/pro/m54-40-s250-r/")
		- [M42-10-S260-R](https://emanual.robotis.com/docs/en/dxl/pro/m42-10-s260-r/ "https://emanual.robotis.com/docs/en/dxl/pro/m42-10-s260-r/")
		- [L54-50-S500-R](https://emanual.robotis.com/docs/en/dxl/pro/l54-50-s500-r/ "https://emanual.robotis.com/docs/en/dxl/pro/l54-50-s500-r/")
		- [L54-50-S290-R](https://emanual.robotis.com/docs/en/dxl/pro/l54-50-s290-r/ "https://emanual.robotis.com/docs/en/dxl/pro/l54-50-s290-r/")
		- [L54-30-S500-R](https://emanual.robotis.com/docs/en/dxl/pro/l54-30-s500-r/ "https://emanual.robotis.com/docs/en/dxl/pro/l54-30-s500-r/")
		- [L54-30-S400-R](https://emanual.robotis.com/docs/en/dxl/pro/l54-30-s400-r/ "https://emanual.robotis.com/docs/en/dxl/pro/l54-30-s400-r/")
		- [L42-10-S300-R](https://emanual.robotis.com/docs/en/dxl/pro/l42-10-s300-r/ "https://emanual.robotis.com/docs/en/dxl/pro/l42-10-s300-r/")
* DYNAMIXEL  SYSTEM
	+ [TurtleBot3](https://emanual.robotis.com/docs/en/platform/turtlebot3/overview/ "https://emanual.robotis.com/docs/en/platform/turtlebot3/overview/")
	+ OpenMANIPULATOR
		- [OpenMANIPULATOR-P](https://emanual.robotis.com/docs/en/platform/openmanipulator_p/overview/ "https://emanual.robotis.com/docs/en/platform/openmanipulator_p/overview/")
		- [OpenMANIPULATOR-X](https://emanual.robotis.com/docs/en/platform/openmanipulator_x/overview/ "https://emanual.robotis.com/docs/en/platform/openmanipulator_x/overview/")
		- [Manipulator-H](https://emanual.robotis.com/docs/en/platform/manipulator_h/introduction/ "https://emanual.robotis.com/docs/en/platform/manipulator_h/introduction/")
	+ Robot Hands
		- [RH-P12-RN(A)](https://emanual.robotis.com/docs/en/platform/rh_p12_rna/ "https://emanual.robotis.com/docs/en/platform/rh_p12_rna/")
		- [RH-P12-RN-UR](https://emanual.robotis.com/docs/en/platform/rh_p12_rn_ur/ "https://emanual.robotis.com/docs/en/platform/rh_p12_rn_ur/")
	+ ROBOTIS OP
		- [ROBOTIS OP3](https://emanual.robotis.com/docs/en/platform/op3/introduction/ "https://emanual.robotis.com/docs/en/platform/op3/introduction/")
		- [ROBOTIS OP](https://emanual.robotis.com/docs/en/platform/op/getting_started/ "https://emanual.robotis.com/docs/en/platform/op/getting_started/")
		- [ROBOTIS OP2](https://emanual.robotis.com/docs/en/platform/op2/getting_started/ "https://emanual.robotis.com/docs/en/platform/op2/getting_started/")
	+ [THORMANG3](https://emanual.robotis.com/docs/en/platform/thormang3/introduction/ "https://emanual.robotis.com/docs/en/platform/thormang3/introduction/")
* EDUCATIONAL  KITS
	+ PLAY
		- [PLAY 300](https://emanual.robotis.com/docs/en/edu/play/play-300/ "https://emanual.robotis.com/docs/en/edu/play/play-300/")
		- [PLAY 600](https://emanual.robotis.com/docs/en/edu/play/play-600/ "https://emanual.robotis.com/docs/en/edu/play/play-600/")
		- [PLAY 700](https://emanual.robotis.com/docs/en/edu/play/play-700/ "https://emanual.robotis.com/docs/en/edu/play/play-700/")
	+ ROBOTIS DREAM II
		- [LEVEL 1](https://emanual.robotis.com/docs/en/edu/dream/dream2-1/ "https://emanual.robotis.com/docs/en/edu/dream/dream2-1/")
		- [LEVEL 2](https://emanual.robotis.com/docs/en/edu/dream/dream2-2/ "https://emanual.robotis.com/docs/en/edu/dream/dream2-2/")
		- [LEVEL 3](https://emanual.robotis.com/docs/en/edu/dream/dream2-3/ "https://emanual.robotis.com/docs/en/edu/dream/dream2-3/")
		- [LEVEL 4](https://emanual.robotis.com/docs/en/edu/dream/dream2-4/ "https://emanual.robotis.com/docs/en/edu/dream/dream2-4/")
		- [LEVEL 5](https://emanual.robotis.com/docs/en/edu/dream/dream2-5/ "https://emanual.robotis.com/docs/en/edu/dream/dream2-5/")
		- [School Set](https://emanual.robotis.com/docs/en/edu/dream/dream2-schoolset/ "https://emanual.robotis.com/docs/en/edu/dream/dream2-schoolset/")
	+ ROBOTIS BIOLOID
		- [ROBOTIS STEM](https://emanual.robotis.com/docs/en/edu/bioloid/stem/ "https://emanual.robotis.com/docs/en/edu/bioloid/stem/")
		- [ROBOTIS PREMIUM](https://emanual.robotis.com/docs/en/edu/bioloid/premium/ "https://emanual.robotis.com/docs/en/edu/bioloid/premium/")
		- [ROBOTIS GP](https://emanual.robotis.com/docs/en/edu/bioloid/gp/ "https://emanual.robotis.com/docs/en/edu/bioloid/gp/")
		- [Beginner Level](https://emanual.robotis.com/docs/en/edu/bioloid/beginner/ "https://emanual.robotis.com/docs/en/edu/bioloid/beginner/")
		- [Comprehensive Level](https://emanual.robotis.com/docs/en/edu/bioloid/comprehensive/ "https://emanual.robotis.com/docs/en/edu/bioloid/comprehensive/")
	+ ROBOTIS ENGINEER
		- [Kit 1](https://emanual.robotis.com/docs/en/edu/engineer/kit1/ "https://emanual.robotis.com/docs/en/edu/engineer/kit1/")
		- [Kit 2](https://emanual.robotis.com/docs/en/edu/engineer/kit2_introduction/ "https://emanual.robotis.com/docs/en/edu/engineer/kit2_introduction/")
	+ [ROBOTIS MINI](https://emanual.robotis.com/docs/en/edu/mini/ "https://emanual.robotis.com/docs/en/edu/mini/")
	+ OLLO 
		- [BUG KIT](https://emanual.robotis.com/docs/en/edu/ollo/bugkit/ "https://emanual.robotis.com/docs/en/edu/ollo/bugkit/")
		- [EXPLORER](https://emanual.robotis.com/docs/en/edu/ollo/explorer/ "https://emanual.robotis.com/docs/en/edu/ollo/explorer/")
		- [INVENTOR](https://emanual.robotis.com/docs/en/edu/ollo/inventor/ "https://emanual.robotis.com/docs/en/edu/ollo/inventor/")
	+ DREAM 
		- [LEVEL 2](https://emanual.robotis.com/docs/en/edu/dream/dream1-2/ "https://emanual.robotis.com/docs/en/edu/dream/dream1-2/")
		- [LEVEL 3](https://emanual.robotis.com/docs/en/edu/dream/dream1-3/ "https://emanual.robotis.com/docs/en/edu/dream/dream1-3/")
		- [LEVEL 4](https://emanual.robotis.com/docs/en/edu/dream/dream1-4/ "https://emanual.robotis.com/docs/en/edu/dream/dream1-4/")
		- [SET A](https://emanual.robotis.com/docs/en/edu/dream/dream-a/ "https://emanual.robotis.com/docs/en/edu/dream/dream-a/")
		- [SET B](https://emanual.robotis.com/docs/en/edu/dream/dream-b/ "https://emanual.robotis.com/docs/en/edu/dream/dream-b/")
* SOFTWARE
	+ DYNAMIXEL
		- [DYNAMIXEL SDK](https://emanual.robotis.com/docs/en/software/dynamixel/dynamixel_sdk/overview/ "https://emanual.robotis.com/docs/en/software/dynamixel/dynamixel_sdk/overview/")
		- [DYNAMIXEL Workbench](https://emanual.robotis.com/docs/en/software/dynamixel/dynamixel_workbench/ "https://emanual.robotis.com/docs/en/software/dynamixel/dynamixel_workbench/")
		- [DYNAMIXEL Wizard 2.0](https://emanual.robotis.com/docs/en/software/dynamixel/dynamixel_wizard2/ "https://emanual.robotis.com/docs/en/software/dynamixel/dynamixel_wizard2/")
	+ Mobile Apps
		- [ROBOTIS MINI](https://emanual.robotis.com/docs/en/software/mobile_app/mini_app/ "https://emanual.robotis.com/docs/en/software/mobile_app/mini_app/")
		- [R+ Block](https://emanual.robotis.com/docs/en/software/rplus2/rplus2_block/ "https://emanual.robotis.com/docs/en/software/rplus2/rplus2_block/")
	+ R+ 1.0
		- [R+ Task](https://emanual.robotis.com/docs/en/software/rplus1/task/getting_started/ "https://emanual.robotis.com/docs/en/software/rplus1/task/getting_started/")
		- [R+ Manager](https://emanual.robotis.com/docs/en/software/rplus1/manager/ "https://emanual.robotis.com/docs/en/software/rplus1/manager/")
		- [R+ Motion](https://emanual.robotis.com/docs/en/software/rplus1/motion/ "https://emanual.robotis.com/docs/en/software/rplus1/motion/")
		- [Dynamixel Wizard](https://emanual.robotis.com/docs/en/software/rplus1/dynamixel_wizard/ "https://emanual.robotis.com/docs/en/software/rplus1/dynamixel_wizard/")
	+ R+ 2.0
		- [R+ Task 2.0](https://emanual.robotis.com/docs/en/software/rplus2/task/ "https://emanual.robotis.com/docs/en/software/rplus2/task/")
		- [R+ Manager 2.0](https://emanual.robotis.com/docs/en/software/rplus2/manager/ "https://emanual.robotis.com/docs/en/software/rplus2/manager/")
		- [R+ Motion 2.0](https://emanual.robotis.com/docs/en/software/rplus2/motion/ "https://emanual.robotis.com/docs/en/software/rplus2/motion/")
		- [R+ Design 2.0](https://emanual.robotis.com/docs/en/software/rplus2/design/ "https://emanual.robotis.com/docs/en/software/rplus2/design/")
		- [R+ Scratch](https://emanual.robotis.com/docs/en/software/rplus2/scratch/ "https://emanual.robotis.com/docs/en/software/rplus2/scratch/")
	+ [R+ Task 3.0](https://emanual.robotis.com/docs/en/software/rplustask3/ "https://emanual.robotis.com/docs/en/software/rplustask3/")
	+ R+ Mobile
		- [R+ m.Task 2.0](https://emanual.robotis.com/docs/en/software/rplus_mobile/mtask20/ "https://emanual.robotis.com/docs/en/software/rplus_mobile/mtask20/")
		- [R+ m.Motion 2.0](https://emanual.robotis.com/docs/en/software/rplus_mobile/mmotion/ "https://emanual.robotis.com/docs/en/software/rplus_mobile/mmotion/")
		- [R+ m.Design](https://emanual.robotis.com/docs/en/software/rplus_mobile/mdesign/ "https://emanual.robotis.com/docs/en/software/rplus_mobile/mdesign/")
	+ EMBEDDED SDK
		- [Embedded C(CM-510/700)](https://emanual.robotis.com/docs/en/software/embedded_sdk/embedded_c_cm510/ "https://emanual.robotis.com/docs/en/software/embedded_sdk/embedded_c_cm510/")
		- [Embedded C(CM-530)](https://emanual.robotis.com/docs/en/software/embedded_sdk/embedded_c_cm530/ "https://emanual.robotis.com/docs/en/software/embedded_sdk/embedded_c_cm530/")
		- [ZIGBEE SDK](https://emanual.robotis.com/docs/en/software/embedded_sdk/zigbee_sdk/ "https://emanual.robotis.com/docs/en/software/embedded_sdk/zigbee_sdk/")
	+ [Arduino IDE](https://emanual.robotis.com/docs/en/software/arduino_ide/ "https://emanual.robotis.com/docs/en/software/arduino_ide/")
	+ [ROBOTIS Framework Packages](https://emanual.robotis.com/docs/en/software/robotis_framework_packages/ "https://emanual.robotis.com/docs/en/software/robotis_framework_packages/")
	+ [ROBOTIS Manipulator library](https://emanual.robotis.com/docs/en/software/robotis_manipulator_libs/ "https://emanual.robotis.com/docs/en/software/robotis_manipulator_libs/")
	+ [\*OpenCM IDE](https://emanual.robotis.com/docs/en/software/opencm_ide/getting_started/ "https://emanual.robotis.com/docs/en/software/opencm_ide/getting_started/")
* PARTS
	+ Controller
		- [CM-50](https://emanual.robotis.com/docs/en/parts/controller/cm-50/ "https://emanual.robotis.com/docs/en/parts/controller/cm-50/")
		- [CM-150](https://emanual.robotis.com/docs/en/parts/controller/cm-150/ "https://emanual.robotis.com/docs/en/parts/controller/cm-150/")
		- [CM-151](https://emanual.robotis.com/docs/en/parts/controller/cm-151/ "https://emanual.robotis.com/docs/en/parts/controller/cm-151/")
		- [CM-200](https://emanual.robotis.com/docs/en/parts/controller/cm-200/ "https://emanual.robotis.com/docs/en/parts/controller/cm-200/")
		- [CM-530](https://emanual.robotis.com/docs/en/parts/controller/cm-530/ "https://emanual.robotis.com/docs/en/parts/controller/cm-530/")
		- [CM-550](https://emanual.robotis.com/docs/en/parts/controller/cm-550/ "https://emanual.robotis.com/docs/en/parts/controller/cm-550/")
		- [CM-700](https://emanual.robotis.com/docs/en/parts/controller/cm-700/ "https://emanual.robotis.com/docs/en/parts/controller/cm-700/")
		- [OpenRB-150](https://emanual.robotis.com/docs/en/parts/controller/openrb-150/ "https://emanual.robotis.com/docs/en/parts/controller/openrb-150/")
		- [OpenCM9.04](https://emanual.robotis.com/docs/en/parts/controller/opencm904/ "https://emanual.robotis.com/docs/en/parts/controller/opencm904/")
		- [OpenCM 485 EXP](https://emanual.robotis.com/docs/en/parts/controller/opencm485exp/ "https://emanual.robotis.com/docs/en/parts/controller/opencm485exp/")
		- [OpenCR1.0](https://emanual.robotis.com/docs/en/parts/controller/opencr10/ "https://emanual.robotis.com/docs/en/parts/controller/opencr10/")
		- [CM-100A](https://emanual.robotis.com/docs/en/parts/controller/cm-100/ "https://emanual.robotis.com/docs/en/parts/controller/cm-100/")
		- [CM-5](https://emanual.robotis.com/docs/en/parts/controller/cm-5/ "https://emanual.robotis.com/docs/en/parts/controller/cm-5/")
		- [CM-510](https://emanual.robotis.com/docs/en/parts/controller/cm-510/ "https://emanual.robotis.com/docs/en/parts/controller/cm-510/")
		- [CM-900](https://emanual.robotis.com/docs/en/parts/controller/cm-900/ "https://emanual.robotis.com/docs/en/parts/controller/cm-900/")
	+ Communication
		- [RC-100A/100B](https://emanual.robotis.com/docs/en/parts/communication/rc-100/ "https://emanual.robotis.com/docs/en/parts/communication/rc-100/")
		- [RC-200](https://emanual.robotis.com/docs/en/parts/communication/rc-200/ "https://emanual.robotis.com/docs/en/parts/communication/rc-200/")
		- [BT-210](https://emanual.robotis.com/docs/en/parts/communication/bt-210/ "https://emanual.robotis.com/docs/en/parts/communication/bt-210/")
		- [BT-410](https://emanual.robotis.com/docs/en/parts/communication/bt-410/ "https://emanual.robotis.com/docs/en/parts/communication/bt-410/")
		- [BT-410 Dongle](https://emanual.robotis.com/docs/en/parts/communication/bt-410-dongle/ "https://emanual.robotis.com/docs/en/parts/communication/bt-410-dongle/")
		- [ZIG-100/110A](https://emanual.robotis.com/docs/en/parts/communication/zig-110/ "https://emanual.robotis.com/docs/en/parts/communication/zig-110/")
		- [BT-100/110A](https://emanual.robotis.com/docs/en/parts/communication/bt-110/ "https://emanual.robotis.com/docs/en/parts/communication/bt-110/")
		- [ZIG2Serial](https://emanual.robotis.com/docs/en/parts/communication/zig2serial/ "https://emanual.robotis.com/docs/en/parts/communication/zig2serial/")
	+ Motors
		- [Geared Motor](https://emanual.robotis.com/docs/en/parts/motor/gm-10a/ "https://emanual.robotis.com/docs/en/parts/motor/gm-10a/")
		- [Servo Motor](https://emanual.robotis.com/docs/en/parts/motor/servo_motor/ "https://emanual.robotis.com/docs/en/parts/motor/servo_motor/")
		- [High Speed Geared Motor](https://emanual.robotis.com/docs/en/parts/motor/h_speed_geared_motor/ "https://emanual.robotis.com/docs/en/parts/motor/h_speed_geared_motor/")
		- [Low Speed Geared Motor](https://emanual.robotis.com/docs/en/parts/motor/l_speed_geared_motor/ "https://emanual.robotis.com/docs/en/parts/motor/l_speed_geared_motor/")
	+ Interface
		- [DYNAMIXEL Communication Bridge](https://emanual.robotis.com/docs/en/parts/interface/dxl_bridge/ "https://emanual.robotis.com/docs/en/parts/interface/dxl_bridge/")
		- [LN-101](https://emanual.robotis.com/docs/en/parts/interface/ln-101/ "https://emanual.robotis.com/docs/en/parts/interface/ln-101/")
		- [U2D2](https://emanual.robotis.com/docs/en/parts/interface/u2d2/ "https://emanual.robotis.com/docs/en/parts/interface/u2d2/")
		- [U2D2 Power Hub](https://emanual.robotis.com/docs/en/parts/interface/u2d2_power_hub/ "https://emanual.robotis.com/docs/en/parts/interface/u2d2_power_hub/")
		- [DYNAMIXEL Shield](https://emanual.robotis.com/docs/en/parts/interface/dynamixel_shield/ "https://emanual.robotis.com/docs/en/parts/interface/dynamixel_shield/")
		- [DYNAMIXEL Shield MKR](https://emanual.robotis.com/docs/en/parts/interface/mkr_shield/ "https://emanual.robotis.com/docs/en/parts/interface/mkr_shield/")
		- [USB2DYNAMIXEL](https://emanual.robotis.com/docs/en/parts/interface/usb2dynamixel/ "https://emanual.robotis.com/docs/en/parts/interface/usb2dynamixel/")
	+ Sensors
		- [IR Sensor](https://emanual.robotis.com/docs/en/parts/sensor/irss-10/ "https://emanual.robotis.com/docs/en/parts/sensor/irss-10/")
		- [Distance Sensor](https://emanual.robotis.com/docs/en/parts/sensor/dms-80/ "https://emanual.robotis.com/docs/en/parts/sensor/dms-80/")
		- [Touch Sensor](https://emanual.robotis.com/docs/en/parts/sensor/ts-10/ "https://emanual.robotis.com/docs/en/parts/sensor/ts-10/")
		- [Gyro Sensor](https://emanual.robotis.com/docs/en/parts/sensor/gs-12/ "https://emanual.robotis.com/docs/en/parts/sensor/gs-12/")
		- [IR Array Sensor](https://emanual.robotis.com/docs/en/parts/sensor/ir-array/ "https://emanual.robotis.com/docs/en/parts/sensor/ir-array/")
		- [Color Sensor](https://emanual.robotis.com/docs/en/parts/sensor/cs-10/ "https://emanual.robotis.com/docs/en/parts/sensor/cs-10/")
		- [Magnetic Sensor](https://emanual.robotis.com/docs/en/parts/sensor/mgss-10/ "https://emanual.robotis.com/docs/en/parts/sensor/mgss-10/")
		- [Temperature Sensor](https://emanual.robotis.com/docs/en/parts/sensor/tps-10/ "https://emanual.robotis.com/docs/en/parts/sensor/tps-10/")
		- [Motion Sensor](https://emanual.robotis.com/docs/en/parts/sensor/pir-10/ "https://emanual.robotis.com/docs/en/parts/sensor/pir-10/")
		- [Integrated Sensor](https://emanual.robotis.com/docs/en/parts/sensor/ax-s1/ "https://emanual.robotis.com/docs/en/parts/sensor/ax-s1/")
	+ Display
		- [LED Module](https://emanual.robotis.com/docs/en/parts/display/lm-10/ "https://emanual.robotis.com/docs/en/parts/display/lm-10/")
* FAQ
	+ [DYNAMIXEL Selection Guide](https://emanual.robotis.com/docs/en/reference/dxl-selection-guide/ "https://emanual.robotis.com/docs/en/reference/dxl-selection-guide/")
	+ [DYNAMIXEL Quick Start Guide](https://emanual.robotis.com/docs/en/dxl/dxl-quick-start-guide/ "https://emanual.robotis.com/docs/en/dxl/dxl-quick-start-guide/")
	+ [DYNAMIXEL](https://emanual.robotis.com/docs/en/faq/faq_dynamixel/ "https://emanual.robotis.com/docs/en/faq/faq_dynamixel/")
	+ [DYNAMIXEL SYSTEM](https://emanual.robotis.com/docs/en/faq/faq_platform/ "https://emanual.robotis.com/docs/en/faq/faq_platform/")
	+ [EDUCATION KITS](https://emanual.robotis.com/docs/en/faq/faq_steam/ "https://emanual.robotis.com/docs/en/faq/faq_steam/")
	+ [SOFTWARE](https://emanual.robotis.com/docs/en/faq/faq_software/ "https://emanual.robotis.com/docs/en/faq/faq_software/")
	+ [PARTS](https://emanual.robotis.com/docs/en/faq/faq_parts/ "https://emanual.robotis.com/docs/en/faq/faq_parts/")
	+ [GENERAL](https://emanual.robotis.com/docs/en/faq/general "https://emanual.robotis.com/docs/en/faq/general")

 DYNAMIXEL

[P Series](https://emanual.robotis.com/docs/en/dxl/p/ "https://emanual.robotis.com/docs/en/dxl/p/")

[PH54-200-S500-R (H54P)](https://emanual.robotis.com/docs/en/dxl/p/ph54-200-s500-r/ "https://emanual.robotis.com/docs/en/dxl/p/ph54-200-s500-r/")
[PH54-100-S500-R (H54P)](https://emanual.robotis.com/docs/en/dxl/p/ph54-100-s500-r/ "https://emanual.robotis.com/docs/en/dxl/p/ph54-100-s500-r/")
[PH42-020-S300-R (H42P)](https://emanual.robotis.com/docs/en/dxl/p/ph42-020-s300-r/ "https://emanual.robotis.com/docs/en/dxl/p/ph42-020-s300-r/")
[PM54-060-S250-R (M54P)](https://emanual.robotis.com/docs/en/dxl/p/pm54-060-s250-r/ "https://emanual.robotis.com/docs/en/dxl/p/pm54-060-s250-r/")
[PM54-040-S250-R (M54P)](https://emanual.robotis.com/docs/en/dxl/p/pm54-040-s250-r/ "https://emanual.robotis.com/docs/en/dxl/p/pm54-040-s250-r/")
[PM42-010-S260-R (M42P)](https://emanual.robotis.com/docs/en/dxl/p/pm42-010-s260-r/ "https://emanual.robotis.com/docs/en/dxl/p/pm42-010-s260-r/")

[DYNAMIXEL DRIVE (DYD)](https://emanual.robotis.com/docs/en/all-dyd/ "https://emanual.robotis.com/docs/en/all-dyd/")

[DYD-11](https://emanual.robotis.com/docs/en/all-dyd/dyd-11 "https://emanual.robotis.com/docs/en/all-dyd/dyd-11")
[DYD-14](https://emanual.robotis.com/docs/en/all-dyd/dyd-14 "https://emanual.robotis.com/docs/en/all-dyd/dyd-14")
[DYD-17](https://emanual.robotis.com/docs/en/all-dyd/dyd-17 "https://emanual.robotis.com/docs/en/all-dyd/dyd-17")

[X Series](https://emanual.robotis.com/docs/en/dxl/x/ "https://emanual.robotis.com/docs/en/dxl/x/")

XW Series

[XW540-T140](https://emanual.robotis.com/docs/en/dxl/x/xw540-t140/ "https://emanual.robotis.com/docs/en/dxl/x/xw540-t140/")
[XW540-T260](https://emanual.robotis.com/docs/en/dxl/x/xw540-t260/ "https://emanual.robotis.com/docs/en/dxl/x/xw540-t260/")
[XW430-T200](https://emanual.robotis.com/docs/en/dxl/x/xw430-t200/ "https://emanual.robotis.com/docs/en/dxl/x/xw430-t200/")
[XW430-T333](https://emanual.robotis.com/docs/en/dxl/x/xw430-t333/ "https://emanual.robotis.com/docs/en/dxl/x/xw430-t333/")

XD Series

[XD540-T150](https://emanual.robotis.com/docs/en/dxl/x/xd540-t150/ "https://emanual.robotis.com/docs/en/dxl/x/xd540-t150/")
[XD540-T270](https://emanual.robotis.com/docs/en/dxl/x/xd540-t270/ "https://emanual.robotis.com/docs/en/dxl/x/xd540-t270/")
[XD430-T210](https://emanual.robotis.com/docs/en/dxl/x/xd430-t210/ "https://emanual.robotis.com/docs/en/dxl/x/xd430-t210/")
[XD430-T350](https://emanual.robotis.com/docs/en/dxl/x/xd430-t350/ "https://emanual.robotis.com/docs/en/dxl/x/xd430-t350/")

XH Series

[XH540-W150](https://emanual.robotis.com/docs/en/dxl/x/xh540-w150/ "https://emanual.robotis.com/docs/en/dxl/x/xh540-w150/")
[XH540-W270](https://emanual.robotis.com/docs/en/dxl/x/xh540-w270/ "https://emanual.robotis.com/docs/en/dxl/x/xh540-w270/")
[XH540-V150](https://emanual.robotis.com/docs/en/dxl/x/xh540-v150/ "https://emanual.robotis.com/docs/en/dxl/x/xh540-v150/")
[XH540-V270](https://emanual.robotis.com/docs/en/dxl/x/xh540-v270/ "https://emanual.robotis.com/docs/en/dxl/x/xh540-v270/")
[XH430-W210](https://emanual.robotis.com/docs/en/dxl/x/xh430-w210/ "https://emanual.robotis.com/docs/en/dxl/x/xh430-w210/")
[XH430-W350](https://emanual.robotis.com/docs/en/dxl/x/xh430-w350/ "https://emanual.robotis.com/docs/en/dxl/x/xh430-w350/")
[XH430-V210](https://emanual.robotis.com/docs/en/dxl/x/xh430-v210/ "https://emanual.robotis.com/docs/en/dxl/x/xh430-v210/")
[XH430-V350](https://emanual.robotis.com/docs/en/dxl/x/xh430-v350/ "https://emanual.robotis.com/docs/en/dxl/x/xh430-v350/")

XM Series

[XM540-W150](https://emanual.robotis.com/docs/en/dxl/x/xm540-w150/ "https://emanual.robotis.com/docs/en/dxl/x/xm540-w150/")
[XM540-W270](https://emanual.robotis.com/docs/en/dxl/x/xm540-w270/ "https://emanual.robotis.com/docs/en/dxl/x/xm540-w270/")
[XM430-W210](https://emanual.robotis.com/docs/en/dxl/x/xm430-w210/ "https://emanual.robotis.com/docs/en/dxl/x/xm430-w210/")
[XM430-W350](https://emanual.robotis.com/docs/en/dxl/x/xm430-w350/ "https://emanual.robotis.com/docs/en/dxl/x/xm430-w350/")

XC Series

[2XC430-W250](https://emanual.robotis.com/docs/en/dxl/x/2xc430-w250/ "https://emanual.robotis.com/docs/en/dxl/x/2xc430-w250/")
[XC430-W150](https://emanual.robotis.com/docs/en/dxl/x/xc430-w150/ "https://emanual.robotis.com/docs/en/dxl/x/xc430-w150/")
[XC430-W240](https://emanual.robotis.com/docs/en/dxl/x/xc430-w240/ "https://emanual.robotis.com/docs/en/dxl/x/xc430-w240/")
[XC430-T150BB](https://emanual.robotis.com/docs/en/dxl/x/xc430-t150bb/ "https://emanual.robotis.com/docs/en/dxl/x/xc430-t150bb/")
[XC430-T240BB](https://emanual.robotis.com/docs/en/dxl/x/xc430-t240bb/ "https://emanual.robotis.com/docs/en/dxl/x/xc430-t240bb/")
[XC330-T288](https://emanual.robotis.com/docs/en/dxl/x/xc330-t288/ "https://emanual.robotis.com/docs/en/dxl/x/xc330-t288/")
[XC330-T181](https://emanual.robotis.com/docs/en/dxl/x/xc330-t181/ "https://emanual.robotis.com/docs/en/dxl/x/xc330-t181/")
[XC330-M181](https://emanual.robotis.com/docs/en/dxl/x/xc330-m181/ "https://emanual.robotis.com/docs/en/dxl/x/xc330-m181/")
[XC330-M288](https://emanual.robotis.com/docs/en/dxl/x/xc330-m288/ "https://emanual.robotis.com/docs/en/dxl/x/xc330-m288/")

XL Series

[2XL430-W250](https://emanual.robotis.com/docs/en/dxl/x/2xl430-w250/ "https://emanual.robotis.com/docs/en/dxl/x/2xl430-w250/")
[XL430-W250](https://emanual.robotis.com/docs/en/dxl/x/xl430-w250/ "https://emanual.robotis.com/docs/en/dxl/x/xl430-w250/")
[XL330-M077](https://emanual.robotis.com/docs/en/dxl/x/xl330-m077/ "https://emanual.robotis.com/docs/en/dxl/x/xl330-m077/")
[XL330-M288](https://emanual.robotis.com/docs/en/dxl/x/xl330-m288/ "https://emanual.robotis.com/docs/en/dxl/x/xl330-m288/")
[XL-320](https://emanual.robotis.com/docs/en/dxl/x/xl320/ "https://emanual.robotis.com/docs/en/dxl/x/xl320/")

[MX Series](https://emanual.robotis.com/docs/en/dxl/mx/ "https://emanual.robotis.com/docs/en/dxl/mx/")

[MX-106T/R(2.0)](https://emanual.robotis.com/docs/en/dxl/mx/mx-106-2/ "https://emanual.robotis.com/docs/en/dxl/mx/mx-106-2/")
[MX-64T/R/AT/AR(2.0)](https://emanual.robotis.com/docs/en/dxl/mx/mx-64-2/ "https://emanual.robotis.com/docs/en/dxl/mx/mx-64-2/")
[MX-28T/R/AT/AR(2.0)](https://emanual.robotis.com/docs/en/dxl/mx/mx-28-2/ "https://emanual.robotis.com/docs/en/dxl/mx/mx-28-2/")
[MX-106T/R](https://emanual.robotis.com/docs/en/dxl/mx/mx-106/ "https://emanual.robotis.com/docs/en/dxl/mx/mx-106/")
[MX-64T/R/AT/AR](https://emanual.robotis.com/docs/en/dxl/mx/mx-64/ "https://emanual.robotis.com/docs/en/dxl/mx/mx-64/")
[MX-28T/R/AT/AR](https://emanual.robotis.com/docs/en/dxl/mx/mx-28/ "https://emanual.robotis.com/docs/en/dxl/mx/mx-28/")
[MX-12W](https://emanual.robotis.com/docs/en/dxl/mx/mx-12w/ "https://emanual.robotis.com/docs/en/dxl/mx/mx-12w/")

AX Series

[AX-18F/18A](https://emanual.robotis.com/docs/en/dxl/ax/ax-18a/ "https://emanual.robotis.com/docs/en/dxl/ax/ax-18a/")
[AX-12/12+/12A](https://emanual.robotis.com/docs/en/dxl/ax/ax-12a/ "https://emanual.robotis.com/docs/en/dxl/ax/ax-12a/")
[AX-12W](https://emanual.robotis.com/docs/en/dxl/ax/ax-12w/ "https://emanual.robotis.com/docs/en/dxl/ax/ax-12w/")

[DYNAMIXEL Protocol 1.0](https://emanual.robotis.com/docs/en/dxl/protocol1/ "https://emanual.robotis.com/docs/en/dxl/protocol1/")

[DYNAMIXEL Protocol 2.0](https://emanual.robotis.com/docs/en/dxl/protocol2/ "https://emanual.robotis.com/docs/en/dxl/protocol2/")

EX Series

[EX-106+](https://emanual.robotis.com/docs/en/dxl/ex/ex-106+/ "https://emanual.robotis.com/docs/en/dxl/ex/ex-106+/") 

DX Series

[DX-113](https://emanual.robotis.com/docs/en/dxl/dx/dx-113/ "https://emanual.robotis.com/docs/en/dxl/dx/dx-113/") 
[DX-116](https://emanual.robotis.com/docs/en/dxl/dx/dx-116/ "https://emanual.robotis.com/docs/en/dxl/dx/dx-116/") 
[DX-117](https://emanual.robotis.com/docs/en/dxl/dx/dx-117/ "https://emanual.robotis.com/docs/en/dxl/dx/dx-117/") 

RX Series

[RX-10](https://emanual.robotis.com/docs/en/dxl/rx/rx-10/ "https://emanual.robotis.com/docs/en/dxl/rx/rx-10/") 
[RX-24F](https://emanual.robotis.com/docs/en/dxl/rx/rx-24f/ "https://emanual.robotis.com/docs/en/dxl/rx/rx-24f/") 
[RX-28](https://emanual.robotis.com/docs/en/dxl/rx/rx-28/ "https://emanual.robotis.com/docs/en/dxl/rx/rx-28/") 
[RX-64](https://emanual.robotis.com/docs/en/dxl/rx/rx-64/ "https://emanual.robotis.com/docs/en/dxl/rx/rx-64/") 

[PRO Series](https://emanual.robotis.com/docs/en/dxl/pro/ "https://emanual.robotis.com/docs/en/dxl/pro/") 

[H54-200-S500-R(A)](https://emanual.robotis.com/docs/en/dxl/pro/h54-200-s500-ra/ "https://emanual.robotis.com/docs/en/dxl/pro/h54-200-s500-ra/") 
[H54-100-S500-R(A)](https://emanual.robotis.com/docs/en/dxl/pro/h54-100-s500-ra/ "https://emanual.robotis.com/docs/en/dxl/pro/h54-100-s500-ra/") 
[H42-20-S300-R(A)](https://emanual.robotis.com/docs/en/dxl/pro/h42-20-s300-ra/ "https://emanual.robotis.com/docs/en/dxl/pro/h42-20-s300-ra/") 
[M54-60-S250-R(A)](https://emanual.robotis.com/docs/en/dxl/pro/m54-60-s250-ra/ "https://emanual.robotis.com/docs/en/dxl/pro/m54-60-s250-ra/") 
[M54-40-S250-R(A)](https://emanual.robotis.com/docs/en/dxl/pro/m54-40-s250-ra/ "https://emanual.robotis.com/docs/en/dxl/pro/m54-40-s250-ra/") 
[M42-10-S260-R(A)](https://emanual.robotis.com/docs/en/dxl/pro/m42-10-s260-ra/ "https://emanual.robotis.com/docs/en/dxl/pro/m42-10-s260-ra/") 
[H54-200-S500-R](https://emanual.robotis.com/docs/en/dxl/pro/h54-200-s500-r/ "https://emanual.robotis.com/docs/en/dxl/pro/h54-200-s500-r/") 
[H54-100-S500-R](https://emanual.robotis.com/docs/en/dxl/pro/h54-100-s500-r/ "https://emanual.robotis.com/docs/en/dxl/pro/h54-100-s500-r/") 
[H42-20-S300-R](https://emanual.robotis.com/docs/en/dxl/pro/h42-20-s300-r/ "https://emanual.robotis.com/docs/en/dxl/pro/h42-20-s300-r/") 
[M54-60-S250-R](https://emanual.robotis.com/docs/en/dxl/pro/m54-60-s250-r/ "https://emanual.robotis.com/docs/en/dxl/pro/m54-60-s250-r/") 
[M54-40-S250-R](https://emanual.robotis.com/docs/en/dxl/pro/m54-40-s250-r/ "https://emanual.robotis.com/docs/en/dxl/pro/m54-40-s250-r/") 
[M42-10-S260-R](https://emanual.robotis.com/docs/en/dxl/pro/m42-10-s260-r/ "https://emanual.robotis.com/docs/en/dxl/pro/m42-10-s260-r/") 
[L54-50-S500-R](https://emanual.robotis.com/docs/en/dxl/pro/l54-50-s500-r/ "https://emanual.robotis.com/docs/en/dxl/pro/l54-50-s500-r/") 
[L54-50-S290-R](https://emanual.robotis.com/docs/en/dxl/pro/l54-50-s290-r/ "https://emanual.robotis.com/docs/en/dxl/pro/l54-50-s290-r/") 
[L54-30-S500-R](https://emanual.robotis.com/docs/en/dxl/pro/l54-30-s500-r/ "https://emanual.robotis.com/docs/en/dxl/pro/l54-30-s500-r/") 
[L54-30-S400-R](https://emanual.robotis.com/docs/en/dxl/pro/l54-30-s400-r/ "https://emanual.robotis.com/docs/en/dxl/pro/l54-30-s400-r/") 
[L42-10-S300-R](https://emanual.robotis.com/docs/en/dxl/pro/l42-10-s300-r/ "https://emanual.robotis.com/docs/en/dxl/pro/l42-10-s300-r/") 

 DYNAMIXEL  SYSTEM

[TurtleBot3](https://emanual.robotis.com/docs/en/platform/turtlebot3/overview/ "https://emanual.robotis.com/docs/en/platform/turtlebot3/overview/")

[OpenMANIPULATOR](https://emanual.robotis.com/docs/en/platform/openmanipulator_main "https://emanual.robotis.com/docs/en/platform/openmanipulator_main")

[OpenMANIPULATOR-P](https://emanual.robotis.com/docs/en/platform/openmanipulator_p/overview/ "https://emanual.robotis.com/docs/en/platform/openmanipulator_p/overview/")
[OpenMANIPULATOR-X](https://emanual.robotis.com/docs/en/platform/openmanipulator_x/overview/ "https://emanual.robotis.com/docs/en/platform/openmanipulator_x/overview/")
[Manipulator-H](https://emanual.robotis.com/docs/en/platform/manipulator_h/introduction/ "https://emanual.robotis.com/docs/en/platform/manipulator_h/introduction/") 

Robot Hands

[RH-P12-RN(A)](https://emanual.robotis.com/docs/en/platform/rh_p12_rna/ "https://emanual.robotis.com/docs/en/platform/rh_p12_rna/")
[RH-P12-RN-UR](https://emanual.robotis.com/docs/en/platform/rh_p12_rn_ur/ "https://emanual.robotis.com/docs/en/platform/rh_p12_rn_ur/")

[ROBOTIS OP](https://emanual.robotis.com/docs/en/platform/op/getting_started/ "https://emanual.robotis.com/docs/en/platform/op/getting_started/")

[ROBOTIS OP3](https://emanual.robotis.com/docs/en/platform/op3/introduction/ "https://emanual.robotis.com/docs/en/platform/op3/introduction/")
[ROBOTIS OP](https://emanual.robotis.com/docs/en/platform/op/getting_started/ "https://emanual.robotis.com/docs/en/platform/op/getting_started/") 
[ROBOTIS OP2](https://emanual.robotis.com/docs/en/platform/op2/getting_started/ "https://emanual.robotis.com/docs/en/platform/op2/getting_started/") 

[THORMANG3](https://emanual.robotis.com/docs/en/platform/thormang3/introduction/ "https://emanual.robotis.com/docs/en/platform/thormang3/introduction/")

 EDUCATIONAL  KITS

PLAY

[PLAY 300](https://emanual.robotis.com/docs/en/edu/play/play-300/ "https://emanual.robotis.com/docs/en/edu/play/play-300/")
[PLAY 600](https://emanual.robotis.com/docs/en/edu/play/play-600/ "https://emanual.robotis.com/docs/en/edu/play/play-600/")
[PLAY 700](https://emanual.robotis.com/docs/en/edu/play/play-700/ "https://emanual.robotis.com/docs/en/edu/play/play-700/")

ROBOTIS DREAM II

[LEVEL 1](https://emanual.robotis.com/docs/en/edu/dream/dream2-1/ "https://emanual.robotis.com/docs/en/edu/dream/dream2-1/")
[LEVEL 2](https://emanual.robotis.com/docs/en/edu/dream/dream2-2/ "https://emanual.robotis.com/docs/en/edu/dream/dream2-2/")
[LEVEL 3](https://emanual.robotis.com/docs/en/edu/dream/dream2-3/ "https://emanual.robotis.com/docs/en/edu/dream/dream2-3/")
[LEVEL 4](https://emanual.robotis.com/docs/en/edu/dream/dream2-4/ "https://emanual.robotis.com/docs/en/edu/dream/dream2-4/")
[LEVEL 5](https://emanual.robotis.com/docs/en/edu/dream/dream2-5/ "https://emanual.robotis.com/docs/en/edu/dream/dream2-5/")
[School Set](https://emanual.robotis.com/docs/en/edu/dream/dream2-schoolset/ "https://emanual.robotis.com/docs/en/edu/dream/dream2-schoolset/")

[ROBOTIS BIOLOID](https://emanual.robotis.com/docs/en/edu/bioloid/ "https://emanual.robotis.com/docs/en/edu/bioloid/")

[ROBOTIS STEM](https://emanual.robotis.com/docs/en/edu/bioloid/stem/ "https://emanual.robotis.com/docs/en/edu/bioloid/stem/")
[ROBOTIS PREMIUM](https://emanual.robotis.com/docs/en/edu/bioloid/premium/ "https://emanual.robotis.com/docs/en/edu/bioloid/premium/")
[ROBOTIS GP](https://emanual.robotis.com/docs/en/edu/bioloid/gp/ "https://emanual.robotis.com/docs/en/edu/bioloid/gp/")
[Beginner Level](https://emanual.robotis.com/docs/en/edu/bioloid/beginner/ "https://emanual.robotis.com/docs/en/edu/bioloid/beginner/") 
[Comprehensive Level](https://emanual.robotis.com/docs/en/edu/bioloid/comprehensive/ "https://emanual.robotis.com/docs/en/edu/bioloid/comprehensive/") 

ROBOTIS ENGINEER

[Kit 1](https://emanual.robotis.com/docs/en/edu/engineer/kit1/ "https://emanual.robotis.com/docs/en/edu/engineer/kit1/")
[Kit 2](https://emanual.robotis.com/docs/en/edu/engineer/kit2_introduction/ "https://emanual.robotis.com/docs/en/edu/engineer/kit2_introduction/")

[ROBOTIS MINI](https://emanual.robotis.com/docs/en/edu/mini/ "https://emanual.robotis.com/docs/en/edu/mini/")

[OLLO](https://emanual.robotis.com/docs/en/edu/ollo/ollo-4/ "https://emanual.robotis.com/docs/en/edu/ollo/ollo-4/") 

[BUG KIT](https://emanual.robotis.com/docs/en/edu/ollo/bugkit/ "https://emanual.robotis.com/docs/en/edu/ollo/bugkit/") 
[EXPLORER](https://emanual.robotis.com/docs/en/edu/ollo/explorer/ "https://emanual.robotis.com/docs/en/edu/ollo/explorer/") 
[INVENTOR](https://emanual.robotis.com/docs/en/edu/ollo/inventor/ "https://emanual.robotis.com/docs/en/edu/ollo/inventor/") 

DREAM

[LEVEL 2](https://emanual.robotis.com/docs/en/edu/dream/dream1-2/ "https://emanual.robotis.com/docs/en/edu/dream/dream1-2/") 
[LEVEL 3](https://emanual.robotis.com/docs/en/edu/dream/dream1-3/ "https://emanual.robotis.com/docs/en/edu/dream/dream1-3/") 
[LEVEL 4](https://emanual.robotis.com/docs/en/edu/dream/dream1-4/ "https://emanual.robotis.com/docs/en/edu/dream/dream1-4/") 
[SET A](https://emanual.robotis.com/docs/en/edu/dream/dream-a/ "https://emanual.robotis.com/docs/en/edu/dream/dream-a/") 
[SET B](https://emanual.robotis.com/docs/en/edu/dream/dream-b/ "https://emanual.robotis.com/docs/en/edu/dream/dream-b/") 

 SOFTWARE

DYNAMIXEL

[DYNAMIXEL SDK](https://emanual.robotis.com/docs/en/software/dynamixel/dynamixel_sdk/overview/ "https://emanual.robotis.com/docs/en/software/dynamixel/dynamixel_sdk/overview/")
[DYNAMIXEL Workbench](https://emanual.robotis.com/docs/en/software/dynamixel/dynamixel_workbench/ "https://emanual.robotis.com/docs/en/software/dynamixel/dynamixel_workbench/")
[DYNAMIXEL Wizard 2.0](https://emanual.robotis.com/docs/en/software/dynamixel/dynamixel_wizard2/ "https://emanual.robotis.com/docs/en/software/dynamixel/dynamixel_wizard2/")

Mobile Apps

[ROBOTIS MINI](https://emanual.robotis.com/docs/en/software/mobile_app/mini_app/ "https://emanual.robotis.com/docs/en/software/mobile_app/mini_app/")
[R+ Block](https://emanual.robotis.com/docs/en/software/rplus2/rplus2_block/ "https://emanual.robotis.com/docs/en/software/rplus2/rplus2_block/")

R+ 1.0

[R+ Task](https://emanual.robotis.com/docs/en/software/rplus1/task/getting_started/ "https://emanual.robotis.com/docs/en/software/rplus1/task/getting_started/")
[R+ Manager](https://emanual.robotis.com/docs/en/software/rplus1/manager/ "https://emanual.robotis.com/docs/en/software/rplus1/manager/")
[R+ Motion](https://emanual.robotis.com/docs/en/software/rplus1/motion/ "https://emanual.robotis.com/docs/en/software/rplus1/motion/")
[Dynamixel Wizard](https://emanual.robotis.com/docs/en/software/rplus1/dynamixel_wizard/ "https://emanual.robotis.com/docs/en/software/rplus1/dynamixel_wizard/")

R+ 2.0

[R+ Task 2.0](https://emanual.robotis.com/docs/en/software/rplus2/task/ "https://emanual.robotis.com/docs/en/software/rplus2/task/")
[R+ Manager 2.0](https://emanual.robotis.com/docs/en/software/rplus2/manager/ "https://emanual.robotis.com/docs/en/software/rplus2/manager/")
[R+ Motion 2.0](https://emanual.robotis.com/docs/en/software/rplus2/motion/ "https://emanual.robotis.com/docs/en/software/rplus2/motion/")
[R+ Design 2.0](https://emanual.robotis.com/docs/en/software/rplus2/design/ "https://emanual.robotis.com/docs/en/software/rplus2/design/")
[R+ Scratch](https://emanual.robotis.com/docs/en/software/rplus2/scratch/ "https://emanual.robotis.com/docs/en/software/rplus2/scratch/")

[R+ Task 3.0](https://emanual.robotis.com/docs/en/software/rplustask3/ "https://emanual.robotis.com/docs/en/software/rplustask3/")

R+ Mobile

[R+ m.Task 2.0](https://emanual.robotis.com/docs/en/software/rplus_mobile/mtask20/ "https://emanual.robotis.com/docs/en/software/rplus_mobile/mtask20/")
[R+ m.Motion 2.0](https://emanual.robotis.com/docs/en/software/rplus_mobile/mmotion/ "https://emanual.robotis.com/docs/en/software/rplus_mobile/mmotion/")
[R+ m.Design](https://emanual.robotis.com/docs/en/software/rplus_mobile/mdesign/ "https://emanual.robotis.com/docs/en/software/rplus_mobile/mdesign/")

[EMBEDDED SDK](https://emanual.robotis.com/docs/en/software/embedded_sdk/ "https://emanual.robotis.com/docs/en/software/embedded_sdk/")

[Embedded C(CM-510/700)](https://emanual.robotis.com/docs/en/software/embedded_sdk/embedded_c_cm510/ "https://emanual.robotis.com/docs/en/software/embedded_sdk/embedded_c_cm510/")
[Embedded C(CM-530)](https://emanual.robotis.com/docs/en/software/embedded_sdk/embedded_c_cm530/ "https://emanual.robotis.com/docs/en/software/embedded_sdk/embedded_c_cm530/")
[ZIGBEE SDK](https://emanual.robotis.com/docs/en/software/embedded_sdk/zigbee_sdk/ "https://emanual.robotis.com/docs/en/software/embedded_sdk/zigbee_sdk/")

[Arduino IDE](https://emanual.robotis.com/docs/en/software/arduino_ide/ "https://emanual.robotis.com/docs/en/software/arduino_ide/")

[ROBOTIS Framework Packages](https://emanual.robotis.com/docs/en/software/robotis_framework_packages/ "https://emanual.robotis.com/docs/en/software/robotis_framework_packages/")

[ROBOTIS Manipulator library](https://emanual.robotis.com/docs/en/software/robotis_manipulator_libs/ "https://emanual.robotis.com/docs/en/software/robotis_manipulator_libs/")

[OpenCM IDE](https://emanual.robotis.com/docs/en/software/opencm_ide/getting_started/ "https://emanual.robotis.com/docs/en/software/opencm_ide/getting_started/") 

 PARTS

[Controller](https://emanual.robotis.com/docs/en/parts/controller/controller_compatibility/ "https://emanual.robotis.com/docs/en/parts/controller/controller_compatibility/")

[CM-50](https://emanual.robotis.com/docs/en/parts/controller/cm-50/ "https://emanual.robotis.com/docs/en/parts/controller/cm-50/")
[CM-150](https://emanual.robotis.com/docs/en/parts/controller/cm-150/ "https://emanual.robotis.com/docs/en/parts/controller/cm-150/")
[CM-151](https://emanual.robotis.com/docs/en/parts/controller/cm-151/ "https://emanual.robotis.com/docs/en/parts/controller/cm-151/")
[CM-200](https://emanual.robotis.com/docs/en/parts/controller/cm-200/ "https://emanual.robotis.com/docs/en/parts/controller/cm-200/")
[CM-530](https://emanual.robotis.com/docs/en/parts/controller/cm-530/ "https://emanual.robotis.com/docs/en/parts/controller/cm-530/")
[CM-550](https://emanual.robotis.com/docs/en/parts/controller/cm-550/ "https://emanual.robotis.com/docs/en/parts/controller/cm-550/")
[CM-700](https://emanual.robotis.com/docs/en/parts/controller/cm-700/ "https://emanual.robotis.com/docs/en/parts/controller/cm-700/")
[OpenRB-150](https://emanual.robotis.com/docs/en/parts/controller/openrb-150/ "https://emanual.robotis.com/docs/en/parts/controller/openrb-150/")
[OpenCM9.04](https://emanual.robotis.com/docs/en/parts/controller/opencm904/ "https://emanual.robotis.com/docs/en/parts/controller/opencm904/")
[OpenCM 485 EXP](https://emanual.robotis.com/docs/en/parts/controller/opencm485exp/ "https://emanual.robotis.com/docs/en/parts/controller/opencm485exp/")
[OpenCR1.0](https://emanual.robotis.com/docs/en/parts/controller/opencr10/ "https://emanual.robotis.com/docs/en/parts/controller/opencr10/")
[CM-100A](https://emanual.robotis.com/docs/en/parts/controller/cm-100/ "https://emanual.robotis.com/docs/en/parts/controller/cm-100/") 
[CM-5](https://emanual.robotis.com/docs/en/parts/controller/cm-5/ "https://emanual.robotis.com/docs/en/parts/controller/cm-5/") 
[CM-510](https://emanual.robotis.com/docs/en/parts/controller/cm-510/ "https://emanual.robotis.com/docs/en/parts/controller/cm-510/") 
[CM-900](https://emanual.robotis.com/docs/en/parts/controller/cm-900/ "https://emanual.robotis.com/docs/en/parts/controller/cm-900/") 

Communication

[RC-100A/100B](https://emanual.robotis.com/docs/en/parts/communication/rc-100/ "https://emanual.robotis.com/docs/en/parts/communication/rc-100/")
[RC-200](https://emanual.robotis.com/docs/en/parts/communication/rc-200/ "https://emanual.robotis.com/docs/en/parts/communication/rc-200/")
[BT-210](https://emanual.robotis.com/docs/en/parts/communication/bt-210/ "https://emanual.robotis.com/docs/en/parts/communication/bt-210/")
[BT-410](https://emanual.robotis.com/docs/en/parts/communication/bt-410/ "https://emanual.robotis.com/docs/en/parts/communication/bt-410/")
[BT-410 Dongle](https://emanual.robotis.com/docs/en/parts/communication/bt-410-dongle/ "https://emanual.robotis.com/docs/en/parts/communication/bt-410-dongle/")
[ZIG-100/110A](https://emanual.robotis.com/docs/en/parts/communication/zig-110/ "https://emanual.robotis.com/docs/en/parts/communication/zig-110/") 
[BT-100/110A](https://emanual.robotis.com/docs/en/parts/communication/bt-110/ "https://emanual.robotis.com/docs/en/parts/communication/bt-110/") 
[ZIG2Serial](https://emanual.robotis.com/docs/en/parts/communication/zig2serial/ "https://emanual.robotis.com/docs/en/parts/communication/zig2serial/") 

Motors

[Geared Motor](https://emanual.robotis.com/docs/en/parts/motor/gm-10a/ "https://emanual.robotis.com/docs/en/parts/motor/gm-10a/")
[Servo Motor](https://emanual.robotis.com/docs/en/parts/motor/servo_motor/ "https://emanual.robotis.com/docs/en/parts/motor/servo_motor/")
[High Speed Geared Motor](https://emanual.robotis.com/docs/en/parts/motor/h_speed_geared_motor/ "https://emanual.robotis.com/docs/en/parts/motor/h_speed_geared_motor/") 
[Low Speed Geared Motor](https://emanual.robotis.com/docs/en/parts/motor/l_speed_geared_motor/ "https://emanual.robotis.com/docs/en/parts/motor/l_speed_geared_motor/") 

Interface

[DYNAMIXEL Communication Bridge](https://emanual.robotis.com/docs/en/parts/interface/dxl_bridge/ "https://emanual.robotis.com/docs/en/parts/interface/dxl_bridge/")
[LN-101](https://emanual.robotis.com/docs/en/parts/interface/ln-101/ "https://emanual.robotis.com/docs/en/parts/interface/ln-101/")
[U2D2](https://emanual.robotis.com/docs/en/parts/interface/u2d2/ "https://emanual.robotis.com/docs/en/parts/interface/u2d2/")
[U2D2 Power Hub](https://emanual.robotis.com/docs/en/parts/interface/u2d2_power_hub/ "https://emanual.robotis.com/docs/en/parts/interface/u2d2_power_hub/")
[DYNAMIXEL Shield](https://emanual.robotis.com/docs/en/parts/interface/dynamixel_shield/ "https://emanual.robotis.com/docs/en/parts/interface/dynamixel_shield/")
[DYNAMIXEL Shield MKR](https://emanual.robotis.com/docs/en/parts/interface/mkr_shield/ "https://emanual.robotis.com/docs/en/parts/interface/mkr_shield/")
[USB2DYNAMIXEL](https://emanual.robotis.com/docs/en/parts/interface/usb2dynamixel/ "https://emanual.robotis.com/docs/en/parts/interface/usb2dynamixel/") 

Sensors

[IR Sensor](https://emanual.robotis.com/docs/en/parts/sensor/irss-10/ "https://emanual.robotis.com/docs/en/parts/sensor/irss-10/")
[Distance Sensor](https://emanual.robotis.com/docs/en/parts/sensor/dms-80/ "https://emanual.robotis.com/docs/en/parts/sensor/dms-80/")
[Touch Sensor](https://emanual.robotis.com/docs/en/parts/sensor/ts-10/ "https://emanual.robotis.com/docs/en/parts/sensor/ts-10/")
[Gyro Sensor](https://emanual.robotis.com/docs/en/parts/sensor/gs-12/ "https://emanual.robotis.com/docs/en/parts/sensor/gs-12/")
[IR Array Sensor](https://emanual.robotis.com/docs/en/parts/sensor/ir-array/ "https://emanual.robotis.com/docs/en/parts/sensor/ir-array/")
[Color Sensor](https://emanual.robotis.com/docs/en/parts/sensor/cs-10/ "https://emanual.robotis.com/docs/en/parts/sensor/cs-10/")
[Magnetic Sensor](https://emanual.robotis.com/docs/en/parts/sensor/mgss-10/ "https://emanual.robotis.com/docs/en/parts/sensor/mgss-10/")
[Temperature Sensor](https://emanual.robotis.com/docs/en/parts/sensor/tps-10/ "https://emanual.robotis.com/docs/en/parts/sensor/tps-10/")
[Motion Sensor](https://emanual.robotis.com/docs/en/parts/sensor/pir-10/ "https://emanual.robotis.com/docs/en/parts/sensor/pir-10/")
[Integrated Sensor](https://emanual.robotis.com/docs/en/parts/sensor/ax-s1/ "https://emanual.robotis.com/docs/en/parts/sensor/ax-s1/") 

Display

[LED Module](https://emanual.robotis.com/docs/en/parts/display/lm-10/ "https://emanual.robotis.com/docs/en/parts/display/lm-10/")

 FAQ

[DYNAMIXEL Selection Guide](https://emanual.robotis.com/docs/en/reference/dxl-selection-guide/ "https://emanual.robotis.com/docs/en/reference/dxl-selection-guide/")

[DYNAMIXEL Quick Start Guide](https://emanual.robotis.com/docs/en/dxl/dxl-quick-start-guide/ "https://emanual.robotis.com/docs/en/dxl/dxl-quick-start-guide/")

[DYNAMIXEL](https://emanual.robotis.com/docs/en/faq/faq_dynamixel/ "https://emanual.robotis.com/docs/en/faq/faq_dynamixel/")

[DYNAMIXEL SYSTEM](https://emanual.robotis.com/docs/en/faq/faq_platform/ "https://emanual.robotis.com/docs/en/faq/faq_platform/")

[EDUCATION KITS](https://emanual.robotis.com/docs/en/faq/faq_steam/ "https://emanual.robotis.com/docs/en/faq/faq_steam/")

[SOFTWARE](https://emanual.robotis.com/docs/en/faq/faq_software/ "https://emanual.robotis.com/docs/en/faq/faq_software/")

[PARTS](https://emanual.robotis.com/docs/en/faq/faq_parts/ "https://emanual.robotis.com/docs/en/faq/faq_parts/")

[GENERAL](https://emanual.robotis.com/docs/en/faq/general "https://emanual.robotis.com/docs/en/faq/general")

![](https://www.google.com/images/cleardot.gif)

 function googleTranslateElementInit() {
 new google.translate.TranslateElement({pageLanguage: 'en', includedLanguages: 'ar,de,es,fr,hi,ja,ko,pt,ru,sv,th,tr,vi,zh-CN', layout: google.translate.TranslateElement.InlineLayout.SIMPLE, multilanguagePage: true, gaTrack: true, gaId: 'UA-83196341'}, 'google\_translate\_element');
 }

N/A

[![](/assets/images/youtube_logo.png)Youtube](https://www.youtube.com/channel/UCDCsF6xgpeb5dAmTK9Usn_g?view_as=subscriber "https://www.youtube.com/channel/UCDCsF6xgpeb5dAmTK9Usn_g?view_as=subscriber")
[![](/assets/images/shop_icon_36.png)  ROBOTIS](http://en.robotis.com "http://en.robotis.com") 
[![](/assets/images/community.jpeg)  Community](https://community.robotis.us/ "https://community.robotis.us/") 
[![](/assets/images/github_logo.png)  GitHub](https://github.com/ROBOTIS-GIT "https://github.com/ROBOTIS-GIT")

[![](/assets/images/youtube_logo.png)](https://www.youtube.com/channel/UCDCsF6xgpeb5dAmTK9Usn_g?view_as=subscriber "https://www.youtube.com/channel/UCDCsF6xgpeb5dAmTK9Usn_g?view_as=subscriber")
[![](/assets/images/shop_icon_28.png)](http://en.robotis.com "http://en.robotis.com")
[![](/assets/images/community.jpeg)](https://community.robotis.us/ "https://community.robotis.us/")
[![](/assets/images/github_logo.png)](https://github.com/ROBOTIS-GIT "https://github.com/ROBOTIS-GIT")

![](/assets/images/search_icon.png)

ToC
[▲  
TOP](# "#")

* [TurtleBot3](https://emanual.robotis.com/docs/en/platform/turtlebot3/locomotion/ "https://emanual.robotis.com/docs/en/platform/turtlebot3/locomotion/")
* 1. Overview
	+ [1. 1. Notices](https://emanual.robotis.com/docs/en/platform/turtlebot3/overview/#notices "https://emanual.robotis.com/docs/en/platform/turtlebot3/overview/#notices")
	+ 1. 2. Events
		- [- Online Competition on RDS](https://emanual.robotis.com/docs/en/platform/turtlebot3/challenges/#online-competition-on-rds "https://emanual.robotis.com/docs/en/platform/turtlebot3/challenges/#online-competition-on-rds")
		- [- Offline Competition](https://emanual.robotis.com/docs/en/platform/turtlebot3/challenges/#offline-competition "https://emanual.robotis.com/docs/en/platform/turtlebot3/challenges/#offline-competition")
		- [- AutoRace RBIZ Challenge](https://emanual.robotis.com/docs/en/platform/turtlebot3/challenges/#autorace-rbiz-challenge "https://emanual.robotis.com/docs/en/platform/turtlebot3/challenges/#autorace-rbiz-challenge")
* 2. Features
	+ [2. 1. Specifications](https://emanual.robotis.com/docs/en/platform/turtlebot3/features/#specifications "https://emanual.robotis.com/docs/en/platform/turtlebot3/features/#specifications")
	+ [2. 2. Components](https://emanual.robotis.com/docs/en/platform/turtlebot3/features/#components "https://emanual.robotis.com/docs/en/platform/turtlebot3/features/#components")
* 3. Quick Start Guide
	+ [3. 1. PC Setup](https://emanual.robotis.com/docs/en/platform/turtlebot3/quick-start/#pc-setup "https://emanual.robotis.com/docs/en/platform/turtlebot3/quick-start/#pc-setup")
	+ [3. 2. SBC Setup](https://emanual.robotis.com/docs/en/platform/turtlebot3/sbc_setup/#sbc-setup "https://emanual.robotis.com/docs/en/platform/turtlebot3/sbc_setup/#sbc-setup")
	+ [3. 3. OpenCR Setup](https://emanual.robotis.com/docs/en/platform/turtlebot3/opencr_setup/#opencr-setup "https://emanual.robotis.com/docs/en/platform/turtlebot3/opencr_setup/#opencr-setup")
	+ [3. 4. Hardware Assembly](https://emanual.robotis.com/docs/en/platform/turtlebot3/hardware_setup/#hardware-assembly "https://emanual.robotis.com/docs/en/platform/turtlebot3/hardware_setup/#hardware-assembly")
	+ [3. 5. Bringup](https://emanual.robotis.com/docs/en/platform/turtlebot3/bringup/#bringup "https://emanual.robotis.com/docs/en/platform/turtlebot3/bringup/#bringup")
	+ 3. 6. Basic Operation
		- [- Teleoperation](https://emanual.robotis.com/docs/en/platform/turtlebot3/basic_operation/#teleoperation "https://emanual.robotis.com/docs/en/platform/turtlebot3/basic_operation/#teleoperation")
		- [- Topic Monitor](https://emanual.robotis.com/docs/en/platform/turtlebot3/basic_operation/#topic-monitor "https://emanual.robotis.com/docs/en/platform/turtlebot3/basic_operation/#topic-monitor")
* 4. SLAM
	+ [4. 1. Run SLAM Node](https://emanual.robotis.com/docs/en/platform/turtlebot3/slam/#run-slam-node "https://emanual.robotis.com/docs/en/platform/turtlebot3/slam/#run-slam-node")
	+ [4. 2. Run Teleoperation Node](https://emanual.robotis.com/docs/en/platform/turtlebot3/slam/#run-teleoperation-node "https://emanual.robotis.com/docs/en/platform/turtlebot3/slam/#run-teleoperation-node")
	+ [4. 3. Tuning Guide](https://emanual.robotis.com/docs/en/platform/turtlebot3/slam/#tuning-guide "https://emanual.robotis.com/docs/en/platform/turtlebot3/slam/#tuning-guide")
	+ [4. 4. Save Map](https://emanual.robotis.com/docs/en/platform/turtlebot3/slam/#save-map "https://emanual.robotis.com/docs/en/platform/turtlebot3/slam/#save-map")
	+ [4. 5. Map](https://emanual.robotis.com/docs/en/platform/turtlebot3/slam/#map "https://emanual.robotis.com/docs/en/platform/turtlebot3/slam/#map")
* 5. Navigation
	+ [5. 1. Run Navigation Nodes](https://emanual.robotis.com/docs/en/platform/turtlebot3/navigation/#run-navigation-nodes "https://emanual.robotis.com/docs/en/platform/turtlebot3/navigation/#run-navigation-nodes")
	+ [5. 2. Estimate Initial Pose](https://emanual.robotis.com/docs/en/platform/turtlebot3/navigation/#estimate-initial-pose "https://emanual.robotis.com/docs/en/platform/turtlebot3/navigation/#estimate-initial-pose")
	+ [5. 3. Set Navigation Goal](https://emanual.robotis.com/docs/en/platform/turtlebot3/navigation/#set-navigation-goal "https://emanual.robotis.com/docs/en/platform/turtlebot3/navigation/#set-navigation-goal")
	+ [5. 4. Tuning Guide](https://emanual.robotis.com/docs/en/platform/turtlebot3/navigation/#tuning-guide "https://emanual.robotis.com/docs/en/platform/turtlebot3/navigation/#tuning-guide")
* 6. Simulation
	+ [6. 1. Gazebo Simulation](https://emanual.robotis.com/docs/en/platform/turtlebot3/simulation/#gazebo-simulation "https://emanual.robotis.com/docs/en/platform/turtlebot3/simulation/#gazebo-simulation")
	+ [6. 2. SLAM Simulation](https://emanual.robotis.com/docs/en/platform/turtlebot3/slam_simulation/ "https://emanual.robotis.com/docs/en/platform/turtlebot3/slam_simulation/")
	+ [6. 3. Navigation Simulation](https://emanual.robotis.com/docs/en/platform/turtlebot3/nav_simulation/ "https://emanual.robotis.com/docs/en/platform/turtlebot3/nav_simulation/")
	+ [6. 4. Fake Node Simulation](https://emanual.robotis.com/docs/en/platform/turtlebot3/fakenode_simulation/ "https://emanual.robotis.com/docs/en/platform/turtlebot3/fakenode_simulation/")
	+ [6. 5. Standalone Gazebo Simulation](https://emanual.robotis.com/docs/en/platform/turtlebot3/standalone_gazebo_simulation/ "https://emanual.robotis.com/docs/en/platform/turtlebot3/standalone_gazebo_simulation/")
* 7. Manipulation
	+ [7. 1. TB3 & OpenMANIPULATOR-X](https://emanual.robotis.com/docs/en/platform/turtlebot3/manipulation/#turtlebot3-with-openmanipulator "https://emanual.robotis.com/docs/en/platform/turtlebot3/manipulation/#turtlebot3-with-openmanipulator")
	+ [7. 2. Software Setup](https://emanual.robotis.com/docs/en/platform/turtlebot3/manipulation/#software-setup "https://emanual.robotis.com/docs/en/platform/turtlebot3/manipulation/#software-setup")
	+ [7. 3. Hardware Assembly](https://emanual.robotis.com/docs/en/platform/turtlebot3/manipulation/#hardware-assembly "https://emanual.robotis.com/docs/en/platform/turtlebot3/manipulation/#hardware-assembly")
	+ [7. 4. OpenCR Setup](https://emanual.robotis.com/docs/en/platform/turtlebot3/manipulation/#opencr-setup "https://emanual.robotis.com/docs/en/platform/turtlebot3/manipulation/#opencr-setup")
	+ [7. 5. Bringup](https://emanual.robotis.com/docs/en/platform/turtlebot3/manipulation/#bringup "https://emanual.robotis.com/docs/en/platform/turtlebot3/manipulation/#bringup")
	+ [7. 6. Simulation](https://emanual.robotis.com/docs/en/platform/turtlebot3/manipulation/#simulation "https://emanual.robotis.com/docs/en/platform/turtlebot3/manipulation/#simulation")
	+ [7. 7. Operate the Actual OpenMANIPULATOR](https://emanual.robotis.com/docs/en/platform/turtlebot3/manipulation/#operate-the-actual-openmanipulator "https://emanual.robotis.com/docs/en/platform/turtlebot3/manipulation/#operate-the-actual-openmanipulator")
	+ [7. 8. SLAM](https://emanual.robotis.com/docs/en/platform/turtlebot3/manipulation/#slam "https://emanual.robotis.com/docs/en/platform/turtlebot3/manipulation/#slam")
	+ [7. 9. Navigation](https://emanual.robotis.com/docs/en/platform/turtlebot3/manipulation/#navigation "https://emanual.robotis.com/docs/en/platform/turtlebot3/manipulation/#navigation")
	+ 7. 10. Home Service Challenge
		- [- Getting Started](https://emanual.robotis.com/docs/en/platform/turtlebot3/home_service_challenge/#getting-started "https://emanual.robotis.com/docs/en/platform/turtlebot3/home_service_challenge/#getting-started")
		- [- Camera Calibration](https://emanual.robotis.com/docs/en/platform/turtlebot3/home_service_challenge/#camera-calibration "https://emanual.robotis.com/docs/en/platform/turtlebot3/home_service_challenge/#camera-calibration")
		- [- SLAM](https://emanual.robotis.com/docs/en/platform/turtlebot3/home_service_challenge/#slam "https://emanual.robotis.com/docs/en/platform/turtlebot3/home_service_challenge/#slam")
		- [- Missions](https://emanual.robotis.com/docs/en/platform/turtlebot3/home_service_challenge/#missions "https://emanual.robotis.com/docs/en/platform/turtlebot3/home_service_challenge/#missions")
		- [- Simulation](https://emanual.robotis.com/docs/en/platform/turtlebot3/home_service_challenge/#simulation "https://emanual.robotis.com/docs/en/platform/turtlebot3/home_service_challenge/#simulation")
* 8. Autonomous Driving
	+ [8. 1. Getting Started](https://emanual.robotis.com/docs/en/platform/turtlebot3/autonomous_driving/#getting-started "https://emanual.robotis.com/docs/en/platform/turtlebot3/autonomous_driving/#getting-started")
	+ 8. 2. Camera Calibration
		- [- Camera Imaging Calibration](https://emanual.robotis.com/docs/en/platform/turtlebot3/autonomous_driving/#camera-imaging-calibration "https://emanual.robotis.com/docs/en/platform/turtlebot3/autonomous_driving/#camera-imaging-calibration")
		- [- Intrinsic Camera Calibration](https://emanual.robotis.com/docs/en/platform/turtlebot3/autonomous_driving/#intrinsic-camera-calibration "https://emanual.robotis.com/docs/en/platform/turtlebot3/autonomous_driving/#intrinsic-camera-calibration")
		- [- Extrinsic Camera Calibration](https://emanual.robotis.com/docs/en/platform/turtlebot3/autonomous_driving/#extrinsic-camera-calibration "https://emanual.robotis.com/docs/en/platform/turtlebot3/autonomous_driving/#extrinsic-camera-calibration")
		- [- Check Calibration Result](https://emanual.robotis.com/docs/en/platform/turtlebot3/autonomous_driving/#check-calibration-result "https://emanual.robotis.com/docs/en/platform/turtlebot3/autonomous_driving/#check-calibration-result")
	+ [8. 3. Lane Detection](https://emanual.robotis.com/docs/en/platform/turtlebot3/autonomous_driving/#lane-detection "https://emanual.robotis.com/docs/en/platform/turtlebot3/autonomous_driving/#lane-detection")
	+ [8. 4. Traffic Sign Detection](https://emanual.robotis.com/docs/en/platform/turtlebot3/autonomous_driving/#traffic-sign-detection "https://emanual.robotis.com/docs/en/platform/turtlebot3/autonomous_driving/#traffic-sign-detection")
	+ 8. 5. Missions
		- [- Traffic Lights](https://emanual.robotis.com/docs/en/platform/turtlebot3/autonomous_driving/#traffic-lights "https://emanual.robotis.com/docs/en/platform/turtlebot3/autonomous_driving/#traffic-lights")
		- [- Intersection](https://emanual.robotis.com/docs/en/platform/turtlebot3/autonomous_driving/#intersection "https://emanual.robotis.com/docs/en/platform/turtlebot3/autonomous_driving/#intersection")
		- [- Construction](https://emanual.robotis.com/docs/en/platform/turtlebot3/autonomous_driving/#construction "https://emanual.robotis.com/docs/en/platform/turtlebot3/autonomous_driving/#construction")
		- [- Parking](https://emanual.robotis.com/docs/en/platform/turtlebot3/autonomous_driving/#parking "https://emanual.robotis.com/docs/en/platform/turtlebot3/autonomous_driving/#parking")
		- [- Level Crossing](https://emanual.robotis.com/docs/en/platform/turtlebot3/autonomous_driving/#level-crossing "https://emanual.robotis.com/docs/en/platform/turtlebot3/autonomous_driving/#level-crossing")
		- [- Tunnel](https://emanual.robotis.com/docs/en/platform/turtlebot3/autonomous_driving/#tunnel "https://emanual.robotis.com/docs/en/platform/turtlebot3/autonomous_driving/#tunnel")
	+ [8. 6. TurtleBot3 AutoRace 2019](https://emanual.robotis.com/docs/en/platform/turtlebot3/autonomous_driving_autorace/ "https://emanual.robotis.com/docs/en/platform/turtlebot3/autonomous_driving_autorace/")
* 9. Machine Learning
	+ [9. 1. Software Setup](https://emanual.robotis.com/docs/en/platform/turtlebot3/machine_learning/#software-setup "https://emanual.robotis.com/docs/en/platform/turtlebot3/machine_learning/#software-setup")
	+ [9. 2. Set parameters](https://emanual.robotis.com/docs/en/platform/turtlebot3/machine_learning/#set-parameters "https://emanual.robotis.com/docs/en/platform/turtlebot3/machine_learning/#set-parameters")
	+ [9. 3. Run Machine Learning](https://emanual.robotis.com/docs/en/platform/turtlebot3/machine_learning/#run-machine-learning "https://emanual.robotis.com/docs/en/platform/turtlebot3/machine_learning/#run-machine-learning")
* 10. Examples
	+ [10. 1. Interactive Markers](https://emanual.robotis.com/docs/en/platform/turtlebot3/basic_examples/#move-using-interactive-markers "https://emanual.robotis.com/docs/en/platform/turtlebot3/basic_examples/#move-using-interactive-markers")
	+ [10. 2. Obstacle Detection](https://emanual.robotis.com/docs/en/platform/turtlebot3/basic_examples/#obstacle-detection "https://emanual.robotis.com/docs/en/platform/turtlebot3/basic_examples/#obstacle-detection")
	+ [10. 3. Position Control](https://emanual.robotis.com/docs/en/platform/turtlebot3/basic_examples/#position-control "https://emanual.robotis.com/docs/en/platform/turtlebot3/basic_examples/#position-control")
	+ [10. 4. Point Operation](https://emanual.robotis.com/docs/en/platform/turtlebot3/basic_examples/#point-operation "https://emanual.robotis.com/docs/en/platform/turtlebot3/basic_examples/#point-operation")
	+ [10. 5. Patrol](https://emanual.robotis.com/docs/en/platform/turtlebot3/basic_examples/#patrol "https://emanual.robotis.com/docs/en/platform/turtlebot3/basic_examples/#patrol")
	+ [10. 6. TurtleBot Follower Demo](https://emanual.robotis.com/docs/en/platform/turtlebot3/basic_examples/#turtlebot-follower-demo "https://emanual.robotis.com/docs/en/platform/turtlebot3/basic_examples/#turtlebot-follower-demo")
	+ [10. 7. TurtleBot Panorama Demo](https://emanual.robotis.com/docs/en/platform/turtlebot3/basic_examples/#turtlebot-panorama-demo "https://emanual.robotis.com/docs/en/platform/turtlebot3/basic_examples/#turtlebot-panorama-demo")
	+ [10. 8. Automatic Parking](https://emanual.robotis.com/docs/en/platform/turtlebot3/basic_examples/#automatic-parking "https://emanual.robotis.com/docs/en/platform/turtlebot3/basic_examples/#automatic-parking")
	+ [10. 9. Automatic Parking Vision](https://emanual.robotis.com/docs/en/platform/turtlebot3/basic_examples/#automatic-parking-vision "https://emanual.robotis.com/docs/en/platform/turtlebot3/basic_examples/#automatic-parking-vision")
	+ [10. 10. Load Multiple TurtleBot3s](https://emanual.robotis.com/docs/en/platform/turtlebot3/basic_examples/#load-multiple-turtlebot3s "https://emanual.robotis.com/docs/en/platform/turtlebot3/basic_examples/#load-multiple-turtlebot3s")
* 11. Friends(Locomotion)
	+ [11. 1. TurtleBot3 Friends: Car](https://emanual.robotis.com/docs/en/platform/turtlebot3/locomotion/#turtlebot3-friends-car "https://emanual.robotis.com/docs/en/platform/turtlebot3/locomotion/#turtlebot3-friends-car")
	+ [11. 2. TurtleBot3 Friends: OpenMANIPULATOR](https://emanual.robotis.com/docs/en/platform/turtlebot3/locomotion/#turtlebot3-friends-openmanipulator "https://emanual.robotis.com/docs/en/platform/turtlebot3/locomotion/#turtlebot3-friends-openmanipulator")
	+ [11. 3. TurtleBot3 Friends: Segway](https://emanual.robotis.com/docs/en/platform/turtlebot3/locomotion/#turtlebot3-friends-segway "https://emanual.robotis.com/docs/en/platform/turtlebot3/locomotion/#turtlebot3-friends-segway")
	+ [11. 4. TurtleBot3 Friends: Conveyor](https://emanual.robotis.com/docs/en/platform/turtlebot3/locomotion/#turtlebot3-friends-conveyor "https://emanual.robotis.com/docs/en/platform/turtlebot3/locomotion/#turtlebot3-friends-conveyor")
	+ [11. 5. TurtleBot3 Friends: Monster](https://emanual.robotis.com/docs/en/platform/turtlebot3/locomotion/#turtlebot3-friends-monster "https://emanual.robotis.com/docs/en/platform/turtlebot3/locomotion/#turtlebot3-friends-monster")
	+ [11. 6. TurtleBot3 Friends: Tank](https://emanual.robotis.com/docs/en/platform/turtlebot3/locomotion/#turtlebot3-friends-tank "https://emanual.robotis.com/docs/en/platform/turtlebot3/locomotion/#turtlebot3-friends-tank")
	+ [11. 7. TurtleBot3 Friends: Omni](https://emanual.robotis.com/docs/en/platform/turtlebot3/locomotion/#turtlebot3-friends-omni "https://emanual.robotis.com/docs/en/platform/turtlebot3/locomotion/#turtlebot3-friends-omni")
	+ [11. 8. TurtleBot3 Friends: Mecanum](https://emanual.robotis.com/docs/en/platform/turtlebot3/locomotion/#turtlebot3-friends-mecanum "https://emanual.robotis.com/docs/en/platform/turtlebot3/locomotion/#turtlebot3-friends-mecanum")
	+ [11. 9. TurtleBot3 Friends: Bike](https://emanual.robotis.com/docs/en/platform/turtlebot3/locomotion/#turtlebot3-friends-bike "https://emanual.robotis.com/docs/en/platform/turtlebot3/locomotion/#turtlebot3-friends-bike")
	+ [11. 10. TurtleBot3 Friends: Road Train](https://emanual.robotis.com/docs/en/platform/turtlebot3/locomotion/#turtlebot3-friends-road-train "https://emanual.robotis.com/docs/en/platform/turtlebot3/locomotion/#turtlebot3-friends-road-train")
	+ [11. 11. TurtleBot3 Friends: Real TurtleBot](https://emanual.robotis.com/docs/en/platform/turtlebot3/locomotion/#turtlebot3-friends-real-turtlebot "https://emanual.robotis.com/docs/en/platform/turtlebot3/locomotion/#turtlebot3-friends-real-turtlebot")
	+ [11. 12. TurtleBot3 Friends: Carrier](https://emanual.robotis.com/docs/en/platform/turtlebot3/locomotion/#turtlebot3-friends-carrier "https://emanual.robotis.com/docs/en/platform/turtlebot3/locomotion/#turtlebot3-friends-carrier")
* 12. Learn
	+ [12. 1. AWS RoboMaker with TurtleBot3](https://emanual.robotis.com/docs/en/platform/turtlebot3/learn/#aws-robomaker-with-turtlebot3 "https://emanual.robotis.com/docs/en/platform/turtlebot3/learn/#aws-robomaker-with-turtlebot3")
	+ [12. 2. Data Collection via Matlab](https://emanual.robotis.com/docs/en/platform/turtlebot3/learn/#data-collection-via-matlab "https://emanual.robotis.com/docs/en/platform/turtlebot3/learn/#data-collection-via-matlab")
	+ [12. 3. TurtleBot3 Blockly](https://emanual.robotis.com/docs/en/platform/turtlebot3/learn/#turtlebot3-blockly "https://emanual.robotis.com/docs/en/platform/turtlebot3/learn/#turtlebot3-blockly")
	+ [12. 4. TurtleBot3 Simulation on ROS Indigo](https://emanual.robotis.com/docs/en/platform/turtlebot3/learn/#turtlebot3-simulation-on-ros-indigo "https://emanual.robotis.com/docs/en/platform/turtlebot3/learn/#turtlebot3-simulation-on-ros-indigo")
	+ [12. 5. Youtube Course](https://emanual.robotis.com/docs/en/platform/turtlebot3/learn/#youtube-course "https://emanual.robotis.com/docs/en/platform/turtlebot3/learn/#youtube-course")
	+ [12. 6. Books](https://emanual.robotis.com/docs/en/platform/turtlebot3/learn/#books "https://emanual.robotis.com/docs/en/platform/turtlebot3/learn/#books")
	+ 12. 7. Videos
		- [- Open Source Team](https://emanual.robotis.com/docs/en/platform/turtlebot3/videos/#videos-from-open-source-team "https://emanual.robotis.com/docs/en/platform/turtlebot3/videos/#videos-from-open-source-team")
		- [- ROBOTIS Channel](https://emanual.robotis.com/docs/en/platform/turtlebot3/videos/#videos-from-robotis-channel "https://emanual.robotis.com/docs/en/platform/turtlebot3/videos/#videos-from-robotis-channel")
		- [- Projects](https://emanual.robotis.com/docs/en/platform/turtlebot3/projects/ "https://emanual.robotis.com/docs/en/platform/turtlebot3/projects/")
* 13. More Info
	+ 13. 1. Appendixes
		- [- DYNAMIXEL](https://emanual.robotis.com/docs/en/platform/turtlebot3/appendixes/#dynamixel "https://emanual.robotis.com/docs/en/platform/turtlebot3/appendixes/#dynamixel")
		- [- OpenCR1.0](https://emanual.robotis.com/docs/en/platform/turtlebot3/appendix_opencr1_0/ "https://emanual.robotis.com/docs/en/platform/turtlebot3/appendix_opencr1_0/")
		- [- LDS-01](https://emanual.robotis.com/docs/en/platform/turtlebot3/appendix_lds_01/ "https://emanual.robotis.com/docs/en/platform/turtlebot3/appendix_lds_01/")
		- [- LDS-02](https://emanual.robotis.com/docs/en/platform/turtlebot3/appendix_lds_02/ "https://emanual.robotis.com/docs/en/platform/turtlebot3/appendix_lds_02/")
		- [- RealSense™](https://emanual.robotis.com/docs/en/platform/turtlebot3/appendix_realsense/ "https://emanual.robotis.com/docs/en/platform/turtlebot3/appendix_realsense/")
		- [- Raspberry Pi Camera](https://emanual.robotis.com/docs/en/platform/turtlebot3/appendix_raspi_cam/ "https://emanual.robotis.com/docs/en/platform/turtlebot3/appendix_raspi_cam/")
	+ [13. 2. Compatible Devices](https://emanual.robotis.com/docs/en/platform/turtlebot3/compatible_devices/ "https://emanual.robotis.com/docs/en/platform/turtlebot3/compatible_devices/")
	+ [13. 3. Additional Sensors](https://emanual.robotis.com/docs/en/platform/turtlebot3/additional_sensors/ "https://emanual.robotis.com/docs/en/platform/turtlebot3/additional_sensors/")
	+ 13. 4. OpenSource and Licenses
		- [- OpenSource Software](https://emanual.robotis.com/docs/en/platform/turtlebot3/opensource/#opensource-software "https://emanual.robotis.com/docs/en/platform/turtlebot3/opensource/#opensource-software")
		- [- OpenSource Hardware](https://emanual.robotis.com/docs/en/platform/turtlebot3/opensource/#opensource-hardware "https://emanual.robotis.com/docs/en/platform/turtlebot3/opensource/#opensource-hardware")
		- [- Software License](https://emanual.robotis.com/docs/en/platform/turtlebot3/opensource/#software-license "https://emanual.robotis.com/docs/en/platform/turtlebot3/opensource/#software-license")
		- [- Hardware License](https://emanual.robotis.com/docs/en/platform/turtlebot3/opensource/#hardware-license "https://emanual.robotis.com/docs/en/platform/turtlebot3/opensource/#hardware-license")
		- [- Documents License](https://emanual.robotis.com/docs/en/platform/turtlebot3/opensource/#documents-license "https://emanual.robotis.com/docs/en/platform/turtlebot3/opensource/#documents-license")
	+ 13. 5. Contact US
		- [- About Open Robotics](https://emanual.robotis.com/docs/en/platform/turtlebot3/contact_us/#about-open-robotics "https://emanual.robotis.com/docs/en/platform/turtlebot3/contact_us/#about-open-robotics")
		- [- About ROBOTIS](https://emanual.robotis.com/docs/en/platform/turtlebot3/contact_us/#about-robotis "https://emanual.robotis.com/docs/en/platform/turtlebot3/contact_us/#about-robotis")
		- [- About OST (Open Source Team)](https://emanual.robotis.com/docs/en/platform/turtlebot3/contact_us/#about-ost-open-source-team "https://emanual.robotis.com/docs/en/platform/turtlebot3/contact_us/#about-ost-open-source-team")
* [14. FAQ](https://emanual.robotis.com/docs/en/platform/turtlebot3/faq/#faq "https://emanual.robotis.com/docs/en/platform/turtlebot3/faq/#faq")

[Edit on GitHub](https://github.com/ROBOTIS-GIT/emanual/blob/master/docs/en/platform/turtlebot3/locomotion.md "https://github.com/ROBOTIS-GIT/emanual/blob/master/docs/en/platform/turtlebot3/locomotion.md") 

# [Locomotion](#locomotion "#locomotion")

The contents in e-Manual are subject to be updated without a prior notice. Therefore, some video may differ from the contents in e-Manual.

In the video, watch how TurtleBot3 can be assembled and reassembled with a few additional parts. The waffle plate which is the biggest part among TurtleBot3 components can be assembled in various sizes and shapes, thanks to its diverse holes for bolts and nuts.

With this openended component, handful of TurtleBot3 friends with various characteristics could be built. You can create a totally new robot that is never seen before. Create a variety of robots based on open hardware and try out the new Locomotion.

**TurtleBot3 Friends List**

* [TurtleBot3 Friends: Car](#turtlebot3-friends-car "#turtlebot3-friends-car")
* [TurtleBot3 Friends: OpenManipulator](#turtlebot3-friends-openmanipulator "#turtlebot3-friends-openmanipulator")
* [TurtleBot3 Friends: Segway](#turtlebot3-friends-segway "#turtlebot3-friends-segway")
* [TurtleBot3 Friends: Conveyor](#turtlebot3-friends-conveyor "#turtlebot3-friends-conveyor")
* [TurtleBot3 Friends: Monster](#turtlebot3-friends-monster "#turtlebot3-friends-monster")
* [TurtleBot3 Friends: Tank](#turtlebot3-friends-tank "#turtlebot3-friends-tank")
* [TurtleBot3 Friends: Omni](#turtlebot3-friends-omni "#turtlebot3-friends-omni")
* [TurtleBot3 Friends: Mecanum](#turtlebot3-friends-mecanum "#turtlebot3-friends-mecanum")
* [TurtleBot3 Friends: Bike](#turtlebot3-friends-bike "#turtlebot3-friends-bike")
* [TurtleBot3 Friends: Road Train](#turtlebot3-friends-road-train "#turtlebot3-friends-road-train")
* [TurtleBot3 Friends: Real TurtleBot](#turtlebot3-friends-real-turtlebot "#turtlebot3-friends-real-turtlebot")
* [TurtleBot3 Friends: Carrier](#turtlebot3-friends-carrier "#turtlebot3-friends-carrier")

**Components List ( BOM )**

**Notice**  

Thank you for your continued interests in the TurtleBot3 Friends.  

The listed products in BOM, sold by third-party distributors, are subject to change or discontinuation without a prior notice.  

In this case, please refer to the 3D modeling file to find compatible products.

**Contact Information**  

If you have questions regarding the TurtleBot3 Friends, contact us via [ROBOTIS Forum](https://forum.robotis.com/ "https://forum.robotis.com/").

| Single Item | Quantity | Set Item | Quantity | Purchase Links |
| --- | --- | --- | --- | --- |
| XL430-W250-T ([e-Manual](/docs/en/dxl/x/xl430-w250/ "/docs/en/dxl/x/xl430-w250/")) | 2 | XL430-W250-T | 2 | [ROBOTIS SHOP](http://en.robotis.com/shop_en/item.php?it_id=902-0135-000 "http://en.robotis.com/shop_en/item.php?it_id=902-0135-000") |
| ROBOT CABLE-X4P 240mm | 2 | ROBOT CABLE-X4P 240mm | 2 | [ROBOTIS SHOP](http://en.robotis.com/shop_en/item.php?it_id=903-0245-000 "http://en.robotis.com/shop_en/item.php?it_id=903-0245-000") |
| OpenCR1.0 ([e-Manual](/docs/en/parts/controller/opencr10/ "/docs/en/parts/controller/opencr10/")) | 1 | OpenCR1.0 | 1 | [ROBOTIS SHOP](http://en.robotis.com/shop_en/item.php?it_id=903-0257-000 "http://en.robotis.com/shop_en/item.php?it_id=903-0257-000") |
| TB3 Waffle Plate-IPL-01 | 6 | TB3 Waffle Plate-IPL-01 | 1 | [ROBOTIS SHOP](http://en.robotis.com/shop_en/item.php?it_id=903-0259-000 "http://en.robotis.com/shop_en/item.php?it_id=903-0259-000") |
| TB3 PCB Support-IBB-01 | 4 | TB3 PCB Support-IBB-01 | 1 | [ROBOTIS SHOP](http://en.robotis.com/shop_en/item.php?it_id=903-0261-000 "http://en.robotis.com/shop_en/item.php?it_id=903-0261-000") |
| Rivet-Mg(n) | 4 | Rivet-Mg(n) | 4 | [ROBOTIS SHOP](http://en.robotis.com/shop_en/item.php?it_id=913-3002-000 "http://en.robotis.com/shop_en/item.php?it_id=913-3002-000") |
| TB3 Wheel/Tire | 2 | TB3 Wheel/Tire Set-ISW-01 | 1 | [ROBOTIS SHOP](http://en.robotis.com/shop_en/item.php?it_id=903-0260-000 "http://en.robotis.com/shop_en/item.php?it_id=903-0260-000") |
| PHS M3x8mm | 20 | None | None | [Online Store](https://us.misumi-ec.com/vona2/detail/110302280810/?HissuCode=NKJ3-8&PNSearch=NKJ3-8&KWSearch=NKJ3-8&searchFlow=results2type "https://us.misumi-ec.com/vona2/detail/110302280810/?HissuCode=NKJ3-8&PNSearch=NKJ3-8&KWSearch=NKJ3-8&searchFlow=results2type") |
| PHS M2.5x8mm | 4 | None | None | [Online Store](https://us.misumi-ec.com/vona2/detail/110302280810/?HissuCode=NKJ2.5-8&PNSearch=NKJ2.5-8&KWSearch=NKJ2.5-8&searchFlow=results2type "https://us.misumi-ec.com/vona2/detail/110302280810/?HissuCode=NKJ2.5-8&PNSearch=NKJ2.5-8&KWSearch=NKJ2.5-8&searchFlow=results2type") |
| PHS M2.5x12mm | 16 | None | None | [Online Store](https://us.misumi-ec.com/vona2/detail/221000802128/?HissuCode=00000000-M2.5X12-SUS&PNSearch=00000000-M2.5X12-SUS&KWSearch=00000000-M2.5X12-SUS&searchFlow=results2type "https://us.misumi-ec.com/vona2/detail/221000802128/?HissuCode=00000000-M2.5X12-SUS&PNSearch=00000000-M2.5X12-SUS&KWSearch=00000000-M2.5X12-SUS&searchFlow=results2type") |
| WB\_M2x4mm | 8 | None | None | [Online Store](https://us.misumi-ec.com/vona2/detail/221000551286/?HissuCode=CSH-ST-M2-4&PNSearch=CSH-ST-M2-4&KWSearch=CSH-ST-M2-4&searchFlow=results2type "https://us.misumi-ec.com/vona2/detail/221000551286/?HissuCode=CSH-ST-M2-4&PNSearch=CSH-ST-M2-4&KWSearch=CSH-ST-M2-4&searchFlow=results2type") |
| WB\_M2.5x20mm | 8 | None | None | [Online Store](https://us.misumi-ec.com/vona2/detail/221000551286/?HissuCode=CSH-ST-M2.5-20&PNSearch=CSH-ST-M2.5-20&KWSearch=CSH-ST-M2.5-20&searchFlow=results2type "https://us.misumi-ec.com/vona2/detail/221000551286/?HissuCode=CSH-ST-M2.5-20&PNSearch=CSH-ST-M2.5-20&KWSearch=CSH-ST-M2.5-20&searchFlow=results2type") |
| NUT\_M2.5 | 4 | None | None | [Online Store](https://us.misumi-ec.com/vona2/detail/110300250540/?HissuCode=SLBNR2.5&PNSearch=SLBNR2.5&KWSearch=SLBNR2.5&searchFlow=results2type "https://us.misumi-ec.com/vona2/detail/110300250540/?HissuCode=SLBNR2.5&PNSearch=SLBNR2.5&KWSearch=SLBNR2.5&searchFlow=results2type") |
| NUT\_M3 | 8 | None | None | [Online Store](https://us.misumi-ec.com/vona2/detail/110300250540/?HissuCode=SLBNR3&PNSearch=SLBNR3&KWSearch=SLBNR3&searchFlow=results2type "https://us.misumi-ec.com/vona2/detail/110300250540/?HissuCode=SLBNR3&PNSearch=SLBNR3&KWSearch=SLBNR3&searchFlow=results2type") |
| SB-S3-45 | 4 | None | None | [Online Store](https://us.misumi-ec.com/vona2/detail/110500137950/?HissuCode=SB-O3-45&searchFlow=results2similartn "https://us.misumi-ec.com/vona2/detail/110500137950/?HissuCode=SB-O3-45&searchFlow=results2similartn") |
| SB-S3-35 | 2 | None | None | [Online Store](https://us.misumi-ec.com/vona2/detail/110500137950/?HissuCode=SB-O3-35&searchFlow=results2similartn "https://us.misumi-ec.com/vona2/detail/110500137950/?HissuCode=SB-O3-35&searchFlow=results2similartn") |
| 3D printing parts(HV Converter) | 4 | None | None | [Onshape](https://cad.onshape.com/documents/f369a265c003df3767a37472/w/42e27bfc98b5e204f5a483fe/e/5b993b12303647b7aa5484f2 "https://cad.onshape.com/documents/f369a265c003df3767a37472/w/42e27bfc98b5e204f5a483fe/e/5b993b12303647b7aa5484f2") |

[BOM spreadsheets](https://docs.google.com/spreadsheets/d/1vbIFMRJMbyOd2D6BtwV1Ow_yBc9xRHktqZhTTmwI_l4/edit#gid=368658157 "https://docs.google.com/spreadsheets/d/1vbIFMRJMbyOd2D6BtwV1Ow_yBc9xRHktqZhTTmwI_l4/edit#gid=368658157")

![](/assets/images/platform/turtlebot3/friends/All_friends.jpg)

* Get source codes and make friends! Go to `Examples` → `turtlebot3` → `turtlebot3_friends` on Arduino IDE for OpenCR.

![](/assets/images/platform/turtlebot3/friends/ide.png)

**NOTE**: Any suggestions and ideas for TurtleBot3 Friends Project are always welcomed. Tell us about your creative TurtleBot3 Friends. We can introduce your friend to the world through this wiki page! E-Mail: ost@robotis.com :)

## [TurtleBot3 Friends: Car](#turtlebot3-friends-car "#turtlebot3-friends-car")

**Note**: 3D modeling for TurtleBot3 Friends: Car is not currently available.

* **Type**: RC Car
* **Features**: About 1:2 gear ratio, differential gears want to make Car be in the Formula E!
* **Components**
	+ Two DYNAMIXEL X 430 Series (One for steering, one for driving)
	+ OpenCR1.0 Board
	+ RC100 Remote Controller with BT410 master-slave Bluetooth modules
	+ TurtleBot3 Chassis and Battery
	+ 6 mm plastic frames
* **Hardware**: Some of the components used in this example are prohibited to be shared in 3D modeling.
* **Software**: `Examples` → `turtlebot3` → `turtlebot3_friends` → `turtlebot3_car`
* **Video**:

## [TurtleBot3 Friends: OpenManipulator](#turtlebot3-friends-openmanipulator "#turtlebot3-friends-openmanipulator")

![](/assets/images/platform/turtlebot3/friends/friends_open_manipulator_waffle_pi.png)

* **Type**: Manipulator 4 DOF + 1 Gripper
* **Features**: Compatible with TurtleBot3 Waffle Pi and it has linear gripper for pick and place.
* **Components**
	+ Four DYNAMIXEL X 430 Series (Four for joints, one for gripper)
	+ OpenCR1.0 Board
	+ 3D printed chassis
* **BOM**: Please refer to the [Parts of OpenManipulator](https://docs.google.com/spreadsheets/d/1h46Vw3amU0FZl3JSRS42BNoAaKeJoDlHAJGMKVe05ts/edit#gid=490107710 "https://docs.google.com/spreadsheets/d/1h46Vw3amU0FZl3JSRS42BNoAaKeJoDlHAJGMKVe05ts/edit#gid=490107710")
* Hardware for TurtleBot3 Waffle Pi + OpenManipulator ([Onshape](https://cad.onshape.com/documents/1535c2d7f05d4986e5ab539c/w/72b49bd8c74a47b010391012/e/454b64d637f42073514486f4 "https://cad.onshape.com/documents/1535c2d7f05d4986e5ab539c/w/72b49bd8c74a47b010391012/e/454b64d637f42073514486f4"), [Thingiverse](https://www.thingiverse.com/thing:3081010 "https://www.thingiverse.com/thing:3081010"))
* Hardware for OpenManipulator ([Onshape](http://www.robotis.com/service/download.php?no=690 "http://www.robotis.com/service/download.php?no=690"), [Thingiverse](https://www.thingiverse.com/thing:3069574 "https://www.thingiverse.com/thing:3069574"))
* [Software](https://github.com/ROBOTIS-GIT/open_manipulator_with_tb3 "https://github.com/ROBOTIS-GIT/open_manipulator_with_tb3") (We are preparing for OpenCR Example)
* [OpenManipulator Wiki](/docs/en/platform/openmanipulator/ "/docs/en/platform/openmanipulator/")
* **Video**:

## [TurtleBot3 Friends: Segway](#turtlebot3-friends-segway "#turtlebot3-friends-segway")

![](/assets/images/platform/turtlebot3/friends/friends_segway.png)

**NOTE**:

* Two DYNAMIXEL X 430 Series are need to be set on PWM Mode.
* Filters library have to be downloaded and includes it arduino IDE. [Github Link](https://github.com/JonHub/Filters "https://github.com/JonHub/Filters")

* **Type**: Segway robot
* **Features**: Balancing with only two DYNAMIXEL’s by applying PID controller.
* **Components**
	+ Two DYNAMIXEL X 430 Series (All for balancing)
	+ OpenCR1.0 Board
	+ TurtleBot3 Chassis and Battery
	+ 3D printed chassis
* [BOM](https://docs.google.com/spreadsheets/d/1vbIFMRJMbyOd2D6BtwV1Ow_yBc9xRHktqZhTTmwI_l4/edit?pli=1#gid=368658157 "https://docs.google.com/spreadsheets/d/1vbIFMRJMbyOd2D6BtwV1Ow_yBc9xRHktqZhTTmwI_l4/edit?pli=1#gid=368658157")
* Hardware ([Onshape](http://www.robotis.com/service/download.php?no=680 "http://www.robotis.com/service/download.php?no=680"), [Thingiverse](https://www.thingiverse.com/thing:3069806 "https://www.thingiverse.com/thing:3069806"))
* **Software**: `Examples` → `turtlebot3` → `turtlebot3_friends` → `turtlebot3_segway`
* **Video**:

## [TurtleBot3 Friends: Conveyor](#turtlebot3-friends-conveyor "#turtlebot3-friends-conveyor")

![](/assets/images/platform/turtlebot3/friends/friends_conveyor.png)

* **Type**: 4 Wheel parallel translation vehicle
* **Features**: 4 Joints and 4 wheels will become a futuristic technology on transportation society by overcoming fuel-consuming mechanics.
* **Components**
	+ Eight DYNAMIXEL X 430 Series (Four for steering, four for driving)
	+ OpenCR1.0 Board
	+ RC100 Remote Controller with BT410 master-slave Bluetooth modules
	+ TurtleBot3 Chassis and Battery
	+ 3D printed chassis
* [BOM](https://docs.google.com/spreadsheets/d/1vbIFMRJMbyOd2D6BtwV1Ow_yBc9xRHktqZhTTmwI_l4/edit?pli=1#gid=0 "https://docs.google.com/spreadsheets/d/1vbIFMRJMbyOd2D6BtwV1Ow_yBc9xRHktqZhTTmwI_l4/edit?pli=1#gid=0")
* Hardware ([Onshape](http://www.robotis.com/service/download.php?no=681 "http://www.robotis.com/service/download.php?no=681"), [Thingiverse](https://www.thingiverse.com/thing:3069758 "https://www.thingiverse.com/thing:3069758"))
* **Software**: `Examples` → `turtlebot3` → `turtlebot3_friends` → `turtlebot3_conveyor`
* **Video**:

## [TurtleBot3 Friends: Monster](#turtlebot3-friends-monster "#turtlebot3-friends-monster")

![](/assets/images/platform/turtlebot3/friends/friends_monster.png)

* **Type**: 4WD Car
* **Features**: 4 Big wheels let it be strong in the rough terrain or even a big difference of elevation.
* **Components**
	+ Four DYNAMIXEL X 430 Series (All for driving)
	+ OpenCR1.0 Board
	+ RC100 Remote Controller with BT410 master-slave Bluetooth modules
	+ TurtleBot3 Chassis and Battery
	+ 3D printed chassis
* [BOM](https://docs.google.com/spreadsheets/d/1vbIFMRJMbyOd2D6BtwV1Ow_yBc9xRHktqZhTTmwI_l4/edit?pli=1#gid=46862313 "https://docs.google.com/spreadsheets/d/1vbIFMRJMbyOd2D6BtwV1Ow_yBc9xRHktqZhTTmwI_l4/edit?pli=1#gid=46862313")
* Hardware ([Onshape](http://www.robotis.com/service/download.php?no=682 "http://www.robotis.com/service/download.php?no=682"), [Thingiverse](https://www.thingiverse.com/thing:3069796 "https://www.thingiverse.com/thing:3069796"))
* **Software**: `Examples` → `turtlebot3` → `turtlebot3_friends` → `turtlebot3_monster`
* **Video**:

## [TurtleBot3 Friends: Tank](#turtlebot3-friends-tank "#turtlebot3-friends-tank")

![](/assets/images/platform/turtlebot3/friends/friends_tank.png)

* **Type**: Caterpillar
* **Features**: Caterpillar units which are connected and assembled on sprocket wheels make it be strong in the rough terrain.
* **Components**
	+ Minimum Two DYNAMIXEL X 430 Series
	+ OpenCR1.0 Board
	+ RC100 Remote Controller with BT410 master-slave Bluetooth modules
	+ TurtleBot3 Chassis and Battery
	+ Caterpillar Unit
	+ 3D printed chassis
* [BOM](https://docs.google.com/spreadsheets/d/1vbIFMRJMbyOd2D6BtwV1Ow_yBc9xRHktqZhTTmwI_l4/edit?pli=1#gid=146214313 "https://docs.google.com/spreadsheets/d/1vbIFMRJMbyOd2D6BtwV1Ow_yBc9xRHktqZhTTmwI_l4/edit?pli=1#gid=146214313")
* Hardware ([Onshape](http://www.robotis.com/service/download.php?no=683 "http://www.robotis.com/service/download.php?no=683"), [Thingiverse](https://www.thingiverse.com/thing:3069755 "https://www.thingiverse.com/thing:3069755"))
* **Software**: `Examples` → `turtlebot3` → `turtlebot3_friends` → `turtlebot3_tank`
	+ When running with Two DYNAMIXEL, a dummy DYNAMIXEL should be designed to assemble the passive wheels.
	+ When running with Four DYNAMIXEL, set left wheels as ID 1 and ID 3(with Secondary ID 1), and right wheels as ID 2 and ID 4(with Secondary ID 2)
* **Video**:

## [TurtleBot3 Friends: Omni](#turtlebot3-friends-omni "#turtlebot3-friends-omni")

![](/assets/images/platform/turtlebot3/friends/friends_omni.png)

* **Type**: Omni wheel
* **Features**: Omni wheels have additional discs around the circumference make it laterally driveable.
* **Components**
	+ Three DYNAMIXEL X 430 Series (All for driving)
	+ OpenCR1.0 Board
	+ RC100 Remote Controller with BT410 master-slave Bluetooth modules
	+ TurtleBot3 Chassis and Battery
	+ 3D printed chassis
* [BOM](https://docs.google.com/spreadsheets/d/1vbIFMRJMbyOd2D6BtwV1Ow_yBc9xRHktqZhTTmwI_l4/edit?pli=1#gid=381353368 "https://docs.google.com/spreadsheets/d/1vbIFMRJMbyOd2D6BtwV1Ow_yBc9xRHktqZhTTmwI_l4/edit?pli=1#gid=381353368")
* Hardware ([Onshape](http://www.robotis.com/service/download.php?no=684 "http://www.robotis.com/service/download.php?no=684"), [Thingiverse](https://www.thingiverse.com/thing:3069809 "https://www.thingiverse.com/thing:3069809"))
* **Software**: `Examples` → `turtlebot3` → `turtlebot3_friends` → `turtlebot3_omni`
* **Video**:

## [TurtleBot3 Friends: Mecanum](#turtlebot3-friends-mecanum "#turtlebot3-friends-mecanum")

![](/assets/images/platform/turtlebot3/friends/friends_mecanum.png)

* **Type**: Mecanum wheel
* **Features**: Mecanum wheels have additional discs around the circumference make it laterally driveable.
* **Components**
	+ Four DYNAMIXEL X 430 Series (All for driving)
	+ OpenCR1.0 Board
	+ RC100 Remote Controller with BT410 master-slave Bluetooth modules
	+ TurtleBot3 Chassis and Battery
	+ 3D printed chassis
* [BOM](https://docs.google.com/spreadsheets/d/1vbIFMRJMbyOd2D6BtwV1Ow_yBc9xRHktqZhTTmwI_l4/edit#gid=953634249 "https://docs.google.com/spreadsheets/d/1vbIFMRJMbyOd2D6BtwV1Ow_yBc9xRHktqZhTTmwI_l4/edit#gid=953634249")
* Hardware ([Onshape](http://www.robotis.com/service/download.php?no=685 "http://www.robotis.com/service/download.php?no=685"), [Thingiverse](https://www.thingiverse.com/thing:3069811 "https://www.thingiverse.com/thing:3069811"))
* **Software**: `Examples` → `turtlebot3` → `turtlebot3_friends` → `turtlebot3_mechanum`
* **Video**: See in the video [TurtleBot3 Friends: Omni](#turtlebot3-friends-omni "#turtlebot3-friends-omni") above.

## [TurtleBot3 Friends: Bike](#turtlebot3-friends-bike "#turtlebot3-friends-bike")

![](/assets/images/platform/turtlebot3/friends/friends_bike.png)

* **Type**: 3-DOF Motorcycle
* **Features**: Cute 3-wheeled bikey reveals its existence on the “Car” film as a brother of the “Car”.
* **Components**
	+ Three DYNAMIXEL X 430 Series (One for steering, two for driving)
	+ OpenCR1.0 Board
	+ RC100 Remote Controller with BT410 master-slave Bluetooth modules
	+ TurtleBot3 Chassis and Battery
	+ 3D printed chassis
* [BOM](https://docs.google.com/spreadsheets/d/1vbIFMRJMbyOd2D6BtwV1Ow_yBc9xRHktqZhTTmwI_l4/edit?pli=1#gid=107279947 "https://docs.google.com/spreadsheets/d/1vbIFMRJMbyOd2D6BtwV1Ow_yBc9xRHktqZhTTmwI_l4/edit?pli=1#gid=107279947")
* Hardware ([Onshape](http://www.robotis.com/service/download.php?no=686 "http://www.robotis.com/service/download.php?no=686"), [Thingiverse](https://www.thingiverse.com/thing:3069817 "https://www.thingiverse.com/thing:3069817"))
* **Software**: `Examples` → `turtlebot3` → `turtlebot3_friends` → `turtlebot3_bike`
* **Video**: See in the videos [TurtleBot3 Friends: Car](#turtlebot3-friends-car "#turtlebot3-friends-car") and [TurtleBot3 Friends: monster](#turtlebot3-friends-monster "#turtlebot3-friends-monster") above.

## [TurtleBot3 Friends: Road Train](#turtlebot3-friends-road-train "#turtlebot3-friends-road-train")

![](/assets/images/platform/turtlebot3/friends/friends_road_train.png)

* **Type**: Road train
* **Features**: Road train can connect vehicles and it can serve various things!
* **Components**
	+ Two DYNAMIXEL X 430 Series (two for driving)
	+ OpenCR1.0 Board
	+ RC100 Remote Controller with BT410 master-slave Bluetooth modules
	+ TurtleBot3 Chassis and Battery
	+ 3D printed chassis
* [BOM](https://docs.google.com/spreadsheets/d/1vbIFMRJMbyOd2D6BtwV1Ow_yBc9xRHktqZhTTmwI_l4/edit#gid=1473715654 "https://docs.google.com/spreadsheets/d/1vbIFMRJMbyOd2D6BtwV1Ow_yBc9xRHktqZhTTmwI_l4/edit#gid=1473715654")
* Hardware ([Onshape](http://www.robotis.com/service/download.php?no=687 "http://www.robotis.com/service/download.php?no=687"), [Thingiverse](https://www.thingiverse.com/thing:3069804 "https://www.thingiverse.com/thing:3069804"))
* **Software**: `Examples` → `turtlebot3` → `turtlebot3_friends` → `turtlebot3_road_train`
* **Video**:

## [TurtleBot3 Friends: Real TurtleBot](#turtlebot3-friends-real-turtlebot "#turtlebot3-friends-real-turtlebot")

![](/assets/images/platform/turtlebot3/friends/friends_real.png)

* **Type**: 8-DOF legged robot (a.k.a. Real TurtleBot)
* **Features**: A real TurtleBot will make most of the fanpics in the turtlebot society!.
* **Components**
	+ Ten DYNAMIXEL X 430 Series (Four for leg joint, another four for shoulder joint, two for head)
	+ OpenCR1.0 Board
	+ RC100 Remote Controller with BT410 master-slave Bluetooth modules
	+ TurtleBot3 Chassis and Battery
	+ 3D printed chassis
* [BOM](https://docs.google.com/spreadsheets/d/1vbIFMRJMbyOd2D6BtwV1Ow_yBc9xRHktqZhTTmwI_l4/edit#gid=1862775028 "https://docs.google.com/spreadsheets/d/1vbIFMRJMbyOd2D6BtwV1Ow_yBc9xRHktqZhTTmwI_l4/edit#gid=1862775028")
* Hardware ([Onshape](http://www.robotis.com/service/download.php?no=688 "http://www.robotis.com/service/download.php?no=688"), [Thingiverse](https://www.thingiverse.com/thing:3069748 "https://www.thingiverse.com/thing:3069748"))
* **Software**: `Examples` → `turtlebot3` → `turtlebot3_friends` → `turtlebot3_realturtlebot`
* **Video**:

## [TurtleBot3 Friends: Carrier](#turtlebot3-friends-carrier "#turtlebot3-friends-carrier")

![](/assets/images/platform/turtlebot3/friends/friends_carrier.png)

* **Type**: 2 Wheel mobile based platform as service robot
* **Features**: 2 wheeled mobile platform will serve whatever you wants.
* **Components**
	+ TurtleBot3 Waffle
	+ 6 supports for 4th layer
	+ 6 supports for 5th layer construction
	+ Extra Waffle Plates in each layers
	+ Customized 3D printed wheels and ball casters
* [BOM](https://docs.google.com/spreadsheets/d/1vbIFMRJMbyOd2D6BtwV1Ow_yBc9xRHktqZhTTmwI_l4/edit#gid=71921372 "https://docs.google.com/spreadsheets/d/1vbIFMRJMbyOd2D6BtwV1Ow_yBc9xRHktqZhTTmwI_l4/edit#gid=71921372")
* Hardware ([Onshape](http://www.robotis.com/service/download.php?no=689 "http://www.robotis.com/service/download.php?no=689"), [Thingiverse](https://www.thingiverse.com/thing:3069815 "https://www.thingiverse.com/thing:3069815"))
* **Software**: `Examples` → `turtlebot3` → `turtlebot3_waffle` → `turtlebot3_waffle`
* **Video**:

 Previous Page
Next Page 

$reference = $(".archive");
$(window).resize(function() {
 var newWidth = $reference.width() \* 0.90;
 if (newWidth > 640) { newWidth = 640; }

 // Resize all videos according to aspect ratio
 $("iframe").each(
 function(index, elem) {
 elem.setAttribute("width", (newWidth).toFixed());
 elem.setAttribute("height", (newWidth\*0.56).toFixed());
 }
 );
// Kick off one resize to fix all videos on page load
}).resize();

© 2023 ROBOTIS. Powered by [Jekyll](http://jekyllrb.com "http://jekyllrb.com") & [Minimal Mistakes](https://mademistakes.com/work/minimal-mistakes-jekyll-theme/ "https://mademistakes.com/work/minimal-mistakes-jekyll-theme/").

 var \_gaq = \_gaq || [];
 \_gaq.push(['\_setAccount', 'UA-83196341-13']);
 \_gaq.push(['\_trackPageview']);

 (function() {
 var ga = document.createElement('script'); ga.type = 'text/javascript'; ga.async = true;
 ga.src = ('https:' == document.location.protocol ? 'https://ssl' : 'http://www') + '.google-analytics.com/ga.js';
 var s = document.getElementsByTagName('script')[0]; s.parentNode.insertBefore(ga, s);
 })();

 window.dataLayer = window.dataLayer || [];
 function gtag(){dataLayer.push(arguments);}
 gtag('js', new Date());

 gtag('config', 'G-155KHSC07K');

