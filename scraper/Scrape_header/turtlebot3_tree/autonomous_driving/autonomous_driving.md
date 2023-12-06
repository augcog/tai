

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

* [TurtleBot3](https://emanual.robotis.com/docs/en/platform/turtlebot3/autonomous_driving/ "https://emanual.robotis.com/docs/en/platform/turtlebot3/autonomous_driving/")
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

[Edit on GitHub](https://github.com/ROBOTIS-GIT/emanual/blob/master/docs/en/platform/turtlebot3/autonomous_driving/autonomous_driving.md "https://github.com/ROBOTIS-GIT/emanual/blob/master/docs/en/platform/turtlebot3/autonomous_driving/autonomous_driving.md") 

Kinetic 
Melodic
Noetic
Dashing
Foxy
Humble
Windows

![](/assets/images/platform/turtlebot3/autonomous_driving/autorace_rbiz_challenge_2017_robots_1.png)

# [Autonomous Driving](#autonomous-driving "#autonomous-driving")

**NOTE**: TurtleBot3 Autorace is only officially supported in ROS Kinetic and Noetic

**NOTE**: TurtleBot3 Autorace is only officially supported in ROS Kinetic and Noetic

**NOTE**: TurtleBot3 Autorace is only officially supported in ROS Kinetic and Noetic

**NOTE**: TurtleBot3 Autorace is only officially supported in ROS Kinetic and Noetic

**NOTE**: TurtleBot3 Autorace is only officially supported in ROS Kinetic and Noetic

## Getting Started

**NOTE**: This instructions were tested on `Ubuntu 16.04` and `ROS Kinetic Kame`.

The contents in e-Manual are subject to be updated without a prior notice. Therefore, some video may differ from the contents in e-Manual.

The following instruction describes how to build the autonomous driving TurtleBot3 on ROS by using AutoRace packages.

### [What you need for Autonomous Driving](#what-you-need-for-autonomous-driving "#what-you-need-for-autonomous-driving")

`TurtleBot3 Burger`

* It is the basic model to use AutoRace packages for the autonomous driving on ROS.
* Provided source codes, AutoRace Packages, are made based on TurtleBot3 Burger.

`Remote PC`

* It communicates with an single board computer (SBC) on Turtlebot3.
* Laptop, desktop, or other devices with ROS 1.

`Raspberry Pi camera module with a camera mount`

* You can use a different module if ROS supports it.
* Source codes provided to calibrate the camera are created based on ([Fisheye Lens](https://www.waveshare.com/rpi-camera-g.htm "https://www.waveshare.com/rpi-camera-g.htm")) module.

`AutoRace tracks and objects`

* Download 3D CAD files for AutoRace tracks, Traffic signs, traffic lights and other objects at [ROBOTIS\_GIT/autorace](https://github.com/ROBOTIS-GIT/autorace_track "https://github.com/ROBOTIS-GIT/autorace_track").
* Download a refree system at [ROBOTIS-GIT/autorace\_referee](https://github.com/ROBOTIS-GIT/autorace_referee "https://github.com/ROBOTIS-GIT/autorace_referee")

### [Install Autorace Packages](#install-autorace-packages "#install-autorace-packages")

The following instructions describes how to install packages and to calibrate camera.

1. Install AutoRace package on both `Remote PC` and `SBC`.

```
$ cd ~/catkin_ws/src/
$ git clone -b kinetic-devel https://github.com/ROBOTIS-GIT/turtlebot3_autorace_2020.git
$ cd ~/catkin_ws && catkin_make

```
2. Install additional dependent packages on `Remote PC`.

```
$ sudo apt-get install ros-kinetic-image-transport ros-kinetic-cv-bridge ros-kinetic-vision-opencv python-opencv libopencv-dev ros-kinetic-image-proc

```
3. You need to [Calibrate a Camera on SBC](#calibrate-a-camera-on-sbc "#calibrate-a-camera-on-sbc").

## Getting Started

**NOTE**

* Autorace package is mainly developed on `Ubuntu 20.04` with `ROS1 Noetic Ninjemys`.
* Autorace package is mainly tested under the Gazebo simulation.
* To simulate given examples properly, complete [Simulation](/docs/en/platform/turtlebot3/simulation/ "/docs/en/platform/turtlebot3/simulation/").

**Tip**: If you have actual TurtleBot3, you can perform up to **[Lane Detection](#lane-detection "#lane-detection")** from our Autonomus Driving package. For more details, clcik expansion note (![](/assets/images/icon_unfold.png)

The contents in e-Manual are subject to be updated without a prior notice. Therefore, some video may differ from the contents in e-Manual.

### [Prerequisites](#prerequisites "#prerequisites")

`Remote PC`

* ROS 1 Noetic installed Laptop or desktop PC.
* This instruction is based on Gazebo simulation, but can be ported to the actual robot later.

![](/assets/images/icon_unfold.png)

### [What you need for Autonomous Driving](#what-you-need-for-autonomous-driving "#what-you-need-for-autonomous-driving")

`TurtleBot3 Burger`

* It is the basic model to use AutoRace packages for the autonomous driving on ROS.
* Provided source codes, AutoRace Packages, are made based on TurtleBot3 Burger.

`Remote PC`

* It communicates with an single board computer (SBC) on Turtlebot3.
* Laptop, desktop, or other devices with ROS 1.

`Raspberry Pi camera module with a camera mount`

* You can use a different module if ROS supports it.
* Source codes provided to calibrate the camera are created based on ([Fisheye Lens](https://www.waveshare.com/rpi-camera-g.htm "https://www.waveshare.com/rpi-camera-g.htm")) module.

`AutoRace tracks and objects`

* Download 3D CAD files for AutoRace tracks, Traffic signs, traffic lights and other objects at [ROBOTIS\_GIT/autorace](https://github.com/ROBOTIS-GIT/autorace_track "https://github.com/ROBOTIS-GIT/autorace_track").
* Download a refree system at [ROBOTIS-GIT/autorace\_referee](https://github.com/ROBOTIS-GIT/autorace_referee "https://github.com/ROBOTIS-GIT/autorace_referee")

### [Install Autorace Packages](#install-autorace-packages "#install-autorace-packages")

1. Install the AutoRace 2020 meta package on `Remote PC`.

```
$ cd ~/catkin_ws/src/
$ git clone -b noetic-devel https://github.com/ROBOTIS-GIT/turtlebot3_autorace_2020.git
$ cd ~/catkin_ws && catkin_make

```
2. Install additional dependent packages on `Remote PC`.

```
$ sudo apt install ros-noetic-image-transport ros-noetic-cv-bridge ros-noetic-vision-opencv python3-opencv libopencv-dev ros-noetic-image-proc

```

![](/assets/images/icon_unfold.png)

The following instructions describes how to install packages and to calibrate camera.

1. Install AutoRace package on both `Remote PC` and `SBC`.

```
$ cd ~/catkin_ws/src/
$ git clone -b feature-raspicam https://github.com/ROBOTIS-GIT/turtlebot3_autorace_2020.git
$ cd ~/catkin_ws && catkin_make

```
2. Install additional dependent packages on `SBC`.
	* Create a swap file to prevent lack of memory in building OpenCV.

	```
	  $ sudo fallocate -l 4G /swapfile
	  $ sudo chmod 600 /swapfile
	  $ sudo mkswap /swapfile
	  $ sudo swapon /swapfile

	```
	* Install required dependencies.

	```
	  $ sudo apt-get update
	  $ sudo apt-get install build-essential cmake gcc g++ git unzip pkg-config
	  $ sudo apt-get install libjpeg-dev libpng-dev libtiff-dev libavcodec-dev libavformat-dev libswscale-dev libgtk2.0-dev libcanberra-gtk\* libxvidcore-dev libx264-dev python3-dev python3-numpy python3-pip libtbb2 libtbb-dev libdc1394-22-dev libv4l-dev v4l-utils libopenblas-dev libatlas-base-dev libblas-dev liblapack-dev gfortran libhdf5-dev libprotobuf-dev libgoogle-glog-dev libgflags-dev protobuf-compiler

	```
	* Build with opencv & opencv\_contrib

	```
	  $ cd ~
	  $ wget -O opencv.zip https://github.com/opencv/opencv/archive/4.5.0.zip
	  $ wget -O opencv_contrib.zip https://github.com/opencv/opencv_contrib/archive/4.5.0.zip

	  $ unzip opencv.zip
	  $ unzip opencv_contrib.zip

	  $ mv opencv-4.5.0 opencv
	  $ mv opencv_contrib-4.5.0 opencv_contrib

	```
	* Create cmake file.

	```
	  $ cd opencv
	  $ mkdir build
	  $ cd build
	  $ cmake -D CMAKE\_BUILD\_TYPE=RELEASE \
	          -D CMAKE\_INSTALL\_PREFIX=/usr/local \
	          -D OPENCV\_EXTRA\_MODULES\_PATH=~/opencv_contrib/modules \
	          -D ENABLE\_NEON=ON \
	          -D BUILD\_TIFF=ON \
	          -D WITH\_FFMPEG=ON \
	          -D WITH\_GSTREAMER=ON \
	          -D WITH\_TBB=ON \
	          -D BUILD\_TBB=ON \
	          -D BUILD\_TESTS=OFF \
	          -D WITH\_EIGEN=OFF \
	          -D WITH\_V4L=ON \
	          -D WITH\_LIBV4L=ON \
	          -D WITH\_VTK=OFF \
	          -D OPENCV\_ENABLE\_NONFREE=ON \
	          -D INSTALL\_C\_EXAMPLES=OFF \
	          -D INSTALL\_PYTHON\_EXAMPLES=OFF \
	          -D BUILD\_NEW\_PYTHON\_SUPPORT=ON \
	          -D BUILD\_opencv\_python3=TRUE \
	          -D OPENCV\_GENERATE\_PKGCONFIG=ON \
	          -D BUILD\_EXAMPLES=OFF ..

	```
	* It will take an hour or two to build.

	```
	  $ cd ~/opencv/build
	  $ make -j4
	  $ sudo make install
	  $ sudo ldconfig
	  $ make clean
	  $ sudo apt-get update

	```
	* Turn off Raspberry Pi, take out the microSD card and edit the config.txt in system-boot section. add start\_x=1 before the enable\_uart=1 line.

	```
	  $ sudo apt install ffmpeg
	  $ ffmpeg -f video4linux2 -s 640x480 -i /dev/video0 -ss 0:0:2 -frames 1 capture_test.jpg

	```
	* Install additional dependent packages

	```
	  $ sudo apt install ros-noetic-cv-camera

	```
3. Install additional dependent packages on `Remote PC`.

```
$ sudo apt install ros-noetic-image-transport ros-noetic-image-transport-plugins ros-noetic-cv-bridge ros-noetic-vision-opencv python3-opencv libopencv-dev ros-noetic-image-proc ros-noetic-cv-camera ros-noetic-camera-calibration 

```

## [Camera Calibration](#camera-calibration "#camera-calibration")

Calibrating the camera is very important for autonomous driving. The following describes how to simply calibrate the camera step by step.

## [Camera Calibration](#camera-calibration "#camera-calibration")

Calibrating the camera is very important for autonomous driving. The following describes how to simply calibrate the camera step by step.

### [Camera Imaging Calibration](#camera-imaging-calibration "#camera-imaging-calibration")

1. Launch roscore on `Remote PC`.

```
$ roscore

```
2. Trigger the camera on `SBC`.

```
$ roslaunch turtlebot3_autorace_${Autorace\_Misson}_camera turtlebot3_autorace_camera_pi.launch

```

**WARNING**: Be sure to specify `${Autorace_Misson}` (i.e, **roslaunch turtlebot3\_autorace\_traffic\_light\_camera turtlebot3\_autorace\_camera\_pi.launch**)
3. Execute rqt\_image\_view on `Remote PC`.

```
$ rqt_image_view

```
4. Select **/camera/image/compressed** (or **/camera/image/**) topic on the check box.

![](/assets/images/platform/turtlebot3/autonomous_driving/tb3_click_compressed.png)
5. Excute rqt\_reconfigure on `Remote PC`.

```
$ rosrun rqt_reconfigure rqt_reconfigure

```
6. Click **camera**, and modify parameter value in order to see clear images from the camera.

![](/assets/images/platform/turtlebot3/autonomous_driving/rqt_reconfigure_camera_yaml_edit_01.png)
7. Open **camera.yaml** file located in **turtlebot3*autorace*[Autorace Misson]\_camera/calibration/camera\_calibration** folder.
8. Write modified values to the file.

![](/assets/images/platform/turtlebot3/autonomous_driving/rqt_reconfigure_camera_yaml_edit_02.png)

### [Camera Imaging Calibration](#camera-imaging-calibration "#camera-imaging-calibration")

Camera image calibration is not required in Gazebo Simulation.

![](/assets/images/icon_unfold.png)

1. Launch roscore on `Remote PC`.

```
$ roscore

```
2. Trigger the camera on `SBC`.

```
$ roslaunch turtlebot3_autorace_camera raspberry_pi_camera_publish.launch

```
3. Execute rqt\_image\_view on `Remote PC`.

```
$ rqt_image_view

```

![](/assets/images/platform/turtlebot3/autonomous_driving/noetic_rpi_rqt_image_view.png)

> 
> rqt\_image view
> 
> 
>

### [Intrinsic Camera Calibration](#intrinsic-camera-calibration "#intrinsic-camera-calibration")

Print a checkerboard on A4 size paper. The checkerboard is used for Intrinsic Camera Calibration.

* The checkerboard is stored at **turtlebot3\_autorace\_camera/data/checkerboard\_for\_calibration.pdf**
* Modify value of parameters in **turtlebot3\_autorace\_camera/launch/turtlebot3\_autorace\_intrinsic\_camera\_calibration.launch**
* For detailed information on the camera calibration, see [Camera Calibration manual](http://wiki.ros.org/camera_calibration "http://wiki.ros.org/camera_calibration") from ROS Wiki.

![](/assets/images/platform/turtlebot3/autonomous_driving/autorace_checkerboard.png)

> 
> Checkerboard
> 
> 
>

1. Launch roscore on `Remote PC`.

```
$ roscore

```
2. Trigger the camera on `SBC`.

```
$ roslaunch turtlebot3_autorace_${Autorace\_Misson}_camera turtlebot3_autorace_camera_pi.launch

```

**WARNING**: Be sure to specify `${Autorace_Misson}` (i.e, **roslaunch turtlebot3\_autorace\_traffic\_light\_camera turtlebot3\_autorace\_camera\_pi.launch**)
3. Run a intrinsic camera calibration launch file on `Remote PC`.

```
$ export AUTO\_IN\_CALIB=calibration
$ export GAZEBO\_MODE=false
$ roslaunch turtlebot3_autorace_${Autorace\_Misson}_camera turtlebot3_autorace_intrinsic_camera_calibration.launch

```
4. Use the checkerboard to calibrate the camera, and click **CALIBRATE**.

![](/assets/images/platform/turtlebot3/autonomous_driving/intrinsic_camera_calibration_test.png)
5. Click **Save** to save the intrinsic calibration data.

![](/assets/images/platform/turtlebot3/autonomous_driving/intrinsic_camera_calibration_capture.png)
6. **calibrationdata.tar.gz** folder will be created at **/tmp** folder.

![](/assets/images/platform/turtlebot3/autonomous_driving/camera_320_240_saved_path_01.png)
7. Extract **calibrationdata.tar.gz** folder, and open **ost.yaml**.

![](/assets/images/platform/turtlebot3/autonomous_driving/open_ost_yaml.png)

> 
> ost.yaml
> 
> 
> 

![](/assets/images/platform/turtlebot3/autonomous_driving/ost_yaml.png)

> 
> Intrinsic Calibration Data in ost.yaml
> 
> 
>
8. Copy and paste the data from **ost.yaml** to **camerav2\_320x240\_30fps.yaml**.

![](/assets/images/platform/turtlebot3/autonomous_driving/open_320_240_30fps.png)

> 
> camerav2\_320x240\_30fps.yaml
> 
> 
> 

![](/assets/images/platform/turtlebot3/autonomous_driving/camerav2_320_240_30fps.png)

> 
> Intrinsic Calibration Data in camerav2\_320x240\_30fps.yaml
> 
> 
>

### [Intrinsic Camera Calibration](#intrinsic-camera-calibration "#intrinsic-camera-calibration")

Intrinsic Camera Calibration is not required in Gazebo simulation.

![](/assets/images/icon_unfold.png) **Click to expand : Intrinsic Camera Calibration with an actual TurtleBot3**

Print a checkerboard on A4 size paper. The checkerboard is used for Intrinsic Camera Calibration.

* The checkerboard is stored at **turtlebot3\_autorace\_camera/data/checkerboard\_for\_calibration.pdf**
* Modify value of parameters in **turtlebot3\_autorace\_camera/launch/intrinsic\_camera\_calibration.launch**
* For detailed information on the camera calibration, see [Camera Calibration manual](http://wiki.ros.org/camera_calibration "http://wiki.ros.org/camera_calibration") from ROS Wiki.

![](/assets/images/platform/turtlebot3/autonomous_driving/autorace_checkerboard.png)

> 
> Checkerboard
> 
> 
>

1. Launch roscore on `Remote PC`.

```
$ roscore

```
2. Trigger the camera on `SBC`.

```
$ roslaunch turtlebot3_autorace_camera raspberry_pi_camera_publish.launch

```
3. Run a intrinsic camera calibration launch file on `Remote PC`.

```
$ roslaunch turtlebot3_autorace_camera intrinsic_camera_calibration.launch mode:=calibration

```
4. Use the checkerboard to calibrate the camera, and click **CALIBRATE**.

![](/assets/images/platform/turtlebot3/autonomous_driving/noetic_rpi_before_intrinsic_calibration.png)
5. Click **Save** to save the intrinsic calibration data.

![](/assets/images/platform/turtlebot3/autonomous_driving/noetic_rpi_after_intrinsic_calibration.png)
6. **calibrationdata.tar.gz** folder will be created at **/tmp** folder.

![](/assets/images/platform/turtlebot3/autonomous_driving/noetic_rpi_calibration_data_tar_gz.png)
7. Extract **calibrationdata.tar.gz** folder, and open **ost.yaml**.

![](/assets/images/platform/turtlebot3/autonomous_driving/noetic_rpi_ost_yaml_file.png)

> 
> ost.yaml
> 
> 
> 

![](/assets/images/platform/turtlebot3/autonomous_driving/noetic_rpi_ost_yaml.png)

> 
> Intrinsic Calibration Data in ost.yaml
> 
> 
>
8. Copy and paste the data from **ost.yaml** to **camerav2\_320x240\_30fps.yaml**.

![](/assets/images/platform/turtlebot3/autonomous_driving/noetic_rpi_camerav2_yaml.png)

> 
> camerav2\_320x240\_30fps.yaml
> 
> 
> 

![](/assets/images/platform/turtlebot3/autonomous_driving/noetic_rpi_camerav2_320x240.png)

> 
> Intrinsic Calibration Data in camerav2\_320x240\_30fps.yaml
> 
> 
>

### [Extrinsic Camera Calibration](#extrinsic-camera-calibration "#extrinsic-camera-calibration")

1. Launch roscore on `Remote PC`.

```
$ roscore

```
2. Trigger the camera on `SBC`.

```
$ roslaunch turtlebot3_autorace_${Autorace\_Misson}_camera turtlebot3_autorace_camera_pi.launch

```

**WARNING**: Be sure to specify `${Autorace_Misson}` (i.e, **roslaunch turtlebot3\_autorace\_traffic\_light\_camera turtlebot3\_autorace\_camera\_pi.launch**)
3. Use the command on `Remote PC`.

```
$ export AUTO\_IN\_CALIB=action
$ export GAZEBO\_MODE=false
$ roslaunch turtlebot3_autorace_${Autorace\_Misson}_camera turtlebot3_autorace_intrinsic_camera_calibration.launch

```
4. Run the extrinsic camera calibration launch file on `Remote PC`.

```
$ export AUTO\_EX\_CALIB=calibration
$ roslaunch turtlebot3_autorace_${Autorace\_Misson}_camera turtlebot3_autorace_extrinsic_camera_calibration.launch

```
5. Execute rqt on `Remote PC`.

```
$ rqt

```
6. Click **plugins** > **visualization** > **Image view**; Multiple windows will be present.
7. Select `/camera/image_extrinsic_calib/compressed` and `/camera/image_projected_compensated` topics on each monitors.

* One of two screens will show an image with a red rectangle box. The other one shows the ground projected view (Bird’s eye view).

![](/assets/images/platform/turtlebot3/autonomous_driving/camera_image_extrinsic_calib_compressed.png)

> 
> `/camera/image_extrinsic_calib/compressed` topic
> 
> 
> 

![](/assets/images/platform/turtlebot3/autonomous_driving/camera_image_projected_compensated.png)

> 
> `/camera/image_projected_compensated` topic
> 
> 
>

1. Excute rqt\_reconfigure on `Remote PC`.

```
$ rosrun rqt_reconfigure rqt_reconfigure

```
2. Adjust parameters in `/camera/image_projection` and `/camera/image_compensation_projection`.

* Change `/camera/image_projection` parameter value. It affects `/camera/image_extrinsic_calib/compressed` topic.
* Intrinsic camera calibration will transform the image surrounded by the red rectangle, and will show the image that looks from over the lane.

![](/assets/images/platform/turtlebot3/autonomous_driving/camera_image_projection_compensation_projection.png)

> 
> rqt\_reconfigure
> 
> 
> 

![](/assets/images/platform/turtlebot3/autonomous_driving/modify_image_projection_image_extrinsic_calib.png)

> 
> Result from parameter modification.
> 
> 
>

### [Extrinsic Camera Calibration](#extrinsic-camera-calibration "#extrinsic-camera-calibration")

1. Open a new terminal on `Remote PC` and launch Gazebo.

```
$ roslaunch turtlebot3_gazebo turtlebot3_autorace_2020.launch

```
2. Open a new terminal and launch the intrinsic camera calibration node.

```
$ roslaunch turtlebot3_autorace_camera intrinsic_camera_calibration.launch

```
3. Open a new terminal and launch the extrinsic camera calibration node.

```
$ roslaunch turtlebot3_autorace_camera extrinsic_camera_calibration.launch mode:=calibration

```
4. Execute rqt on `Remote PC`.

```
$ rqt

```
5. Select **plugins** > **visualization** > **Image view**. Create two image view windows.
6. Select `/camera/image_extrinsic_calib/compressed` topic on one window and `/camera/image_projected_compensated` on the other.
	* The first topic shows an image with a red trapezoidal shape and the latter shows the ground projected view (Bird’s eye view).

	![](/assets/images/platform/turtlebot3/autonomous_driving/noetic_extrinsic_calibration.png)

	> 
	> `/camera/image_extrinsic_calib/compressed` (Left) and `/camera/image_projected_compensated` (Right)
	> 
	> 
	>
7. Excute rqt\_reconfigure on `Remote PC`.

```
$ rosrun rqt_reconfigure rqt_reconfigure

```
8. Adjust parameters in `/camera/image_projection` and `/camera/image_compensation_projection`.
	* Change `/camera/image_projection` parameter value. It affects `/camera/image_extrinsic_calib/compressed` topic.
	* Intrinsic camera calibration modifies the perspective of the image in the red trapezoid.

	![](/assets/images/platform/turtlebot3/autonomous_driving/noetic_extrinsic_calibration_reconfigure.png)

	> 
	> rqt\_reconfigure
	> 
	> 
	>
9. After that, overwrite each values on to the yaml files in **turtlebot3\_autorace\_camera/calibration/extrinsic\_calibration/.** This will save the current calibration parameters so that they can be loaded later.

![](/assets/images/platform/turtlebot3/autonomous_driving/noetic_compensation_yaml.png)

> 
> turtlebot3\_autorace\_camera/calibration/extrinsic\_calibration/`compensation.yaml`
> 
> 
> 

![](/assets/images/platform/turtlebot3/autonomous_driving/noetic_projection_yaml.png)

> 
> turtlebot3\_autorace\_camera/calibration/extrinsic\_calibration/`projection.yaml`
> 
> 
>

![](/assets/images/icon_unfold.png) **Click to expand : Extrinsic Camera Calibration with an actual TurtleBot3**

1. Launch roscore on `Remote PC`.

```
$ roscore

```
2. Trigger the camera on `SBC`.

```
$ roslaunch turtlebot3_autorace_camera raspberry_pi_camera_publish.launch

```
3. Use the command on `Remote PC`.

```
$ roslaunch turtlebot3_autorace_camera intrinsic_camera_calibration.launch mode:=action

```
4. Run the extrinsic camera calibration launch file on `Remote PC`.

```
$ roslaunch turtlebot3_autorace_camera extrinsic_camera_calibration.launch mode:=calibration

```
5. Execute rqt on `Remote PC`.

```
$ rqt

```
6. Click **plugins** > **visualization** > **Image view**; Multiple windows will be present.
7. Select `/camera/image_extrinsic_calib/compressed` and `/camera/image_projected_compensated` topics on each monitors.

* One of two screens will show an image with a red rectangle box. The other one shows the ground projected view (Bird’s eye view).

![](/assets/images/platform/turtlebot3/autonomous_driving/noetic_rpi_before_extrinsic_calibration.png)

> 
> `/camera/image_extrinsic_calib/compressed` topic `/camera/image_projected_compensated` topic
> 
> 
>

1. Excute rqt\_reconfigure on `Remote PC`.

```
$ rosrun rqt_reconfigure rqt_reconfigure

```
2. Adjust parameters in `/camera/image_projection` and `/camera/image_compensation_projection`.

* Change `/camera/image_projection` parameter value. It affects `/camera/image_extrinsic_calib/compressed` topic.
* Intrinsic camera calibration will transform the image surrounded by the red rectangle, and will show the image that looks from over the lane.

![](/assets/images/platform/turtlebot3/autonomous_driving/noetic_rpi_extrinsic_calibration_param.png)

> 
> rqt\_reconfigure
> 
> 
> 

![](/assets/images/platform/turtlebot3/autonomous_driving/noetic_rpi_after_extrinsic_calibration.png)

> 
> Result from parameter modification.
> 
> 
>

### [Check Calibration Result](#check-calibration-result "#check-calibration-result")

When you complete all the camera calibration (Camera Imaging Calibration, Intrinsic Calibration, Extrinsic Calibration), be sure that the calibration is successfully applied to the camera.  

The following instruction describes settings for recognition.

1. Launch roscore on `Remote PC`.

```
$ roscore

```
2. Trigger the camera on `SBC`.

```
$ roslaunch turtlebot3_autorace_${Autorace\_Misson}_camera turtlebot3_autorace_camera_pi.launch

```

**WARNING**: Be sure to specify `${Autorace_Misson}` (i.e, **roslaunch turtlebot3\_autorace\_traffic\_light\_camera turtlebot3\_autorace\_camera\_pi.launch**)
3. Run a intrinsic camera calibration launch file on `Remote PC`.

```
$ export AUTO\_IN\_CALIB=action
$ roslaunch turtlebot3_autorace_${Autorace\_Misson}_camera turtlebot3_autorace_intrinsic_camera_calibration.launch

```
4. Open terminal and use the command on `Remote PC`.

```
$ export AUTO\_EX\_CALIB=action
$ roslaunch turtlebot3_autorace_${Autorace\_Misson}_camera turtlebot3_autorace_extrinsic_camera_calibration.launch

```

From now, the following descriptions will mainly adjust `feature detector / color filter` for object recognition. Every adjustment after here is independent to each other’s process. However, if you want to adjust each parameters in series, complete every adjustment perfectly, then continue to next.

### [Check Calibration Result](#check-calibration-result "#check-calibration-result")

After completing calibrations, run the step by step instructions below on `Remote PC` to check the calibration result.

1. Close all of terminal.
2. Open a new terminal and launch Autorace Gazebo simulation. The `roscore` will be automatically launched with the **roslaunch** command.

```
$ roslaunch turtlebot3_gazebo turtlebot3_autorace_2020.launch

```
3. Open a new terminal and launch the intrinsic calibration node.

```
$ roslaunch turtlebot3_autorace_camera intrinsic_camera_calibration.launch

```
4. Open a new terminal and launch the extrinsic calibration node.

```
$ roslaunch turtlebot3_autorace_camera extrinsic_camera_calibration.launch

```
5. Open a new terminal and launch the rqt image viewer.

```
$ rqt_image_view

```
6. With successful calibration settings, the bird eye view image should appear as below when the `/camera/image_projected_compensated` topic is selected.
![](/assets/images/platform/turtlebot3/autonomous_driving/noetic_camera_calibration_rqt_image_view.png)

![](/assets/images/icon_unfold.png) **Click to expand : Extrinsic Camera Calibration for use of actual TurtleBot3**

When you complete all the camera calibration (Camera Imaging Calibration, Intrinsic Calibration, Extrinsic Calibration), be sure that the calibration is successfully applied to the camera.  

The following instruction describes settings for recognition.

1. Launch roscore on `Remote PC`.

```
$ roscore

```
2. Trigger the camera on `SBC`.

```
$ roslaunch turtlebot3_autorace_camera raspberry_pi_camera_publish.launch

```
3. Run a intrinsic camera calibration launch file on `Remote PC`.

```
$ roslaunch turtlebot3_autorace_camera intrinsic_camera_calibration.launch mode:=action

```
4. Open terminal and use the command on `Remote PC`.

```
$ roslaunch turtlebot3_autorace_camera extrinsic_camera_calibration.launch mode:=action

```
5. Execute rqt on `Remote PC`.

```
$ rqt

```

![](/assets/images/platform/turtlebot3/autonomous_driving/noetic_rpi_extrinsic_calibration_rqt.png)

> 
> rqt\_reconfigure
> 
> 
>

From now, the following descriptions will mainly adjust `feature detector / color filter` for object recognition. Every adjustment after here is independent to each other’s process. However, if you want to adjust each parameters in series, complete every adjustment perfectly, then continue to next.

## [Lane Detection](#lane-detection "#lane-detection")

Lane detection package allows Turtlebot3 to drive between two lanes without external influence.

The following instructions describe how to use the lane detection feature and to calibrate camera via rqt.

1. Place TurtleBot3 between yellow and white lanes.

**NOTE**: Be sure that yellow lane is placed left side of the robot and White lane is placed right side of the robot.
2. Launch roscore on `Remote PC`.

```
$ roscore

```
3. Trigger the camera on `SBC`.

```
$ roslaunch turtlebot3_autorace_${Autorace\_Misson}_camera turtlebot3_autorace_camera_pi.launch

```

**WARNING**: Be sure to specify `${Autorace_Misson}` (i.e, **roslaunch turtlebot3\_autorace\_traffic\_light\_camera turtlebot3\_autorace\_camera\_pi.launch**)
4. Run a intrinsic camera calibration launch file on `Remote PC`.

```
$ export AUTO\_IN\_CALIB=action
$ export GAZEBO\_MODE=false
$ roslaunch turtlebot3_autorace_${Autorace\_Misson}_camera turtlebot3_autorace_intrinsic_camera_calibration.launch

```
5. Run a extrinsic camera calibration launch file on `Remote PC`.

```
$ export AUTO\_EX\_CALIB=action
$ roslaunch turtlebot3_autorace_${Autorace\_Misson}_camera turtlebot3_autorace_extrinsic_camera_calibration.launch

```
6. Run a lane detection launch file on `Remote PC`

```
$ export AUTO\_DT\_CALIB=calibration
$ roslaunch turtlebot3_autorace_${Autorace\_Misson}_detect turtlebot3_autorace_detect_lane.launch

```
7. Execute rqt on `Remote PC`.

```
$ rqt

```
8. Click **plugins** > **visualization** > **Image view**; Multiple windows will be present.
9. Select three topics at each image view: `/detect/image_yellow_lane_marker/compressed`, `/detect/image_lane/compressed`, `/detect/image_white_lane_marker/compressed`

	* Left (Yellow line) and Right (White line) screen show a filtered image.  

	![](/assets/images/platform/turtlebot3/autonomous_driving/rqt_yellow_lane_detection.png)

	> 
	> Image view of `/detect/image_yellow_lane_marker/compressed` topic
	> 
	> 
	>![](/assets/images/platform/turtlebot3/autonomous_driving/rqt_white_lane_detection.png)

> 
> Image view of `/detect/image_white_lane_marker/compressed` topic
> 
> 
> 

	* Center screen is the view of the camera from TurtleBot3.  

	![](/assets/images/platform/turtlebot3/autonomous_driving/rqt_image_lane.png)

	> 
	> Image view of `/detect/image_lane/compressed` topic
	> 
	> 
	>
10. Execute rqt\_reconfigure on `Remote PC`.

```
$ rosrun rqt_reconfigure rqt_reconfigure

```
11. Click **Detect Lane** then adjust parameters to do line color filtering.

![](/assets/images/platform/turtlebot3/autonomous_driving/result_of_ybw_configuration_01.png)

> 
> List of Detect Lane Parameters
> 
> 
> 

![](/assets/images/platform/turtlebot3/autonomous_driving/result_of_ybw_configuration_02.png)

> 
> Filtered Image resulted from adjusting parameters at rqt\_reconfigure
> 
> 
> 

**TIP**: Calibration process of line color filtering is sometimes difficult due to physical environment, such as the luminance of light in the room and etc. To make everything quickly, put the value of **lane.yaml** file located in **turtlebot3*autorace*[Autorace\_Misson]\_detect/param/lane/** on the reconfiguration parameter, then start calibration. Calibrate hue low - high value at first. (1) Hue value means the color, and every colors, like `yellow`, `white`, have their own region of hue value (refer to hsv map). Then calibrate saturation low - high value. (2) Every colors have also their own field of saturation. Finally, calibrate the lightness low - high value. (3) In the source code, however, have auto-adjustment function, so calibrating lightness low value is meaningless. Just put the lightness high value to 255. Clearly filtered line image will give you clear result of the lane.
12. Open **lane.yaml** file located in **turtlebot3*autorace*[Autorace\_Misson]\_detect/param/lane/**. You need to write modified values to the file. This will make the camera set its parameters as you set here from next launching.

![](/assets/images/platform/turtlebot3/autonomous_driving/write_lane_yaml.png)
13. Close both **rqt\_rconfigure** and **turtlebot3\_autorace\_detect\_lane**.
14. Open terminal and use the command on `Remote PC`.

```
$ export AUTO\_DT\_CALIB=action
$ roslaunch turtlebot3_autorace_${Autorace\_Misson}_detect turtlebot3_autorace_detect_lane.launch

```
15. Check if the results come out correctly.

	* Open terminal and use the command on `Remote PC`.
```
$ roslaunch turtlebot3_autorace_${Autorace\_Misson}_control turtlebot3_autorace_control_lane.launch

```

	* Open terminal and use the command on `Remote PC`.
```
$ roslaunch turtlebot3_bringup turtlebot3_robot.launch

```
16. After using the commands, TurtleBot3 will start to run.

## [Lane Detection](#lane-detection "#lane-detection")

Lane detection package that runs on the `Remote PC` receives camera images either from TurtleBot3 or Gazebo simulation to detect driving lanes and to drive the Turtlebot3 along them.  

The following instructions describe how to use and calibrate the lane detection feature via rqt.

1. Place the TurtleBot3 inbetween yellow and white lanes.

**NOTE**: The lane detection filters yellow on the left side while filters white on the right side. Be sure that the yellow lane is on the left side of the robot.
2. Open a new terminal and launch Autorace Gazebo simulation. The `roscore` will be automatically launched with the **roslaunch** command.

```
$ roslaunch turtlebot3_gazebo turtlebot3_autorace_2020.launch

```
3. Open a new terminal and launch the intrinsic calibration node.

```
$ roslaunch turtlebot3_autorace_camera intrinsic_camera_calibration.launch

```
4. Open a new terminal and launch the extrinsic calibration node.

```
$ roslaunch turtlebot3_autorace_camera extrinsic_camera_calibration.launch

```
5. Open a new terminal and launch the lane detection calibration node.

```
$ roslaunch turtlebot3_autorace_detect detect_lane.launch mode:=calibration

```
6. Open a new terminal and launch the rqt.

```
$ rqt

```
7. Launch the rqt image viewer by selecting **Plugins** > **Cisualization** > **Image view**.  

 Multiple rqt plugins can be run.
8. Display three topics at each image viewer
	* `/detect/image_lane/compressed`  

	![](/assets/images/platform/turtlebot3/autonomous_driving/noetic_detect_image_lane.png)
	* `/detect/image_yellow_lane_marker/compressed` : a yellow range color filtered image.  

	![](/assets/images/platform/turtlebot3/autonomous_driving/noetic_detect_yellow_lane.png)
	* `/detect/image_white_lane_marker/compressed` : a white range color filtered image.  

	![](/assets/images/platform/turtlebot3/autonomous_driving/noetic_detect_white_lane.png)
9. Open a new terminal and execute rqt\_reconfigure.

```
$ rosrun rqt_reconfigure rqt_reconfigure

```
10. Click **detect\_lane** then adjust parameters so that yellow and white colors can be filtered properly.

![](/assets/images/platform/turtlebot3/autonomous_driving/noetic_lane_reconfigure.png)

**TIP**: Calibration process of line color filtering is sometimes difficult due to physical environment, such as the luminance of light in the room and etc.  

To make everything quickly, put the value of **lane.yaml** file located in **turtlebot3\_auatorace\_detect/param/lane/** on the reconfiguration parameter, then start calibration.  

Calibrate hue low - high value at first. (1) Hue value means the color, and every colors, like `yellow`, `white`, have their own region of hue value (refer to hsv map).  

Then calibrate saturation low - high value. (2) Every colors have also their own field of saturation.  

Finally, calibrate the lightness low - high value. (3) In the source code, however, have auto-adjustment function, so calibrating lightness low value is meaningless. Just put the lightness high value to 255.  

Clearly filtered line image will give you clear result of the lane.
11. Open **lane.yaml** file located in **turtlebot3\_autorace\_detect/param/lane/**. You need to write modified values to the file. This will make the camera set its parameters as you set here from next launching.

![](/assets/images/platform/turtlebot3/autonomous_driving/noetic_lane_yaml.png)

> 
> Modified lane.yaml file
> 
> 
>
12. Close the terminal or terminate with `Ctrl` + `C` on **rqt\_reconfigure** and **detect\_lane** terminals.
13. Open a new terminal and launch the lane detect node without the calibration option.

```
$ roslaunch turtlebot3_autorace_detect detect_lane.launch

```
14. Open a new terminal and launch the node below to start the lane following operation.

```
$ roslaunch turtlebot3_autorace_driving turtlebot3_autorace_control_lane.launch

```

![](/assets/images/icon_unfold.png) **Click to expand : How to Perform Lane Detection with Actual TurtleBot3?**

Lane detection package allows Turtlebot3 to drive between two lanes without external influence.

The following instructions describe how to use the lane detection feature and to calibrate camera via rqt.

1. Place TurtleBot3 between yellow and white lanes.

**NOTE**: Be sure that yellow lane is placed left side of the robot and White lane is placed right side of the robot.
2. Launch roscore on `Remote PC`.

```
$ roscore

```
3. Trigger the camera on `SBC`.

```
$ roslaunch turtlebot3_autorace_camera raspberry_pi_camera_publish.launch

```
4. Run a intrinsic camera calibration launch file on `Remote PC`.

```
$ roslaunch turtlebot3_autorace_camera intrinsic_camera_calibration.launch mode:=action

```
5. Run a extrinsic camera calibration launch file on `Remote PC`.

```
$ roslaunch turtlebot3_autorace_camera extrinsic_camera_calibration.launch mode:=action

```
6. Run a lane detection launch file on `Remote PC`

```
$ roslaunch turtlebot3_autorace_detect detect_lane.launch mode:=calibration

```
7. Execute rqt on `Remote PC`.

```
$ rqt

```
8. Click **plugins** > **visualization** > **Image view**; Multiple windows will be present.
9. Select three topics at each image view: `/detect/image_yellow_lane_marker/compressed`, `/detect/image_lane/compressed`, `/detect/image_white_lane_marker/compressed`

	* Left (Yellow line) and Right (White line) screen show a filtered image. Center screen is the view of the camera from TurtleBot3.
	![](/assets/images/platform/turtlebot3/autonomous_driving/noetic_rpi_before_detect_lane.png)

	> 
	> Image view of `/detect/image_yellow_lane_marker/compressed` topic , `/detect/image_white_lane_marker/compressed` topic , `/detect/image_lane/compressed` topic
	> 
	> 
	>
10. Execute rqt\_reconfigure on `Remote PC`.

```
$ rosrun rqt_reconfigure rqt_reconfigure

```
11. Click **Detect Lane** then adjust parameters to do line color filtering.

![](/assets/images/platform/turtlebot3/autonomous_driving/noetic_rpi_detect_lane_param.png)

> 
> List of Detect Lane Parameters
> 
> 
> 

![](/assets/images/platform/turtlebot3/autonomous_driving/noetic_rpi_after_detect_lane.png)

> 
> Filtered Image resulted from adjusting parameters at rqt\_reconfigure
> 
> 
> 

**TIP**: Calibration process of line color filtering is sometimes difficult due to physical environment, such as the luminance of light in the room and etc. To make everything quickly, put the value of **lane.yaml** file located in **turtlebot3*autorace*\_detect/param/lane/** on the reconfiguration parameter, then start calibration. Calibrate hue low - high value at first. (1) Hue value means the color, and every colors, like `yellow`, `white`, have their own region of hue value (refer to hsv map). Then calibrate saturation low - high value. (2) Every colors have also their own field of saturation. Finally, calibrate the lightness low - high value. (3) In the source code, however, have auto-adjustment function, so calibrating lightness low value is meaningless. Just put the lightness high value to 255. Clearly filtered line image will give you clear result of the lane.
12. Open **lane.yaml** file located in **turtlebot3*\_autorace*\_detect/param/lane/**. You need to write modified values to the file. This will make the camera set its parameters as you set here from next launching.

![](/assets/images/platform/turtlebot3/autonomous_driving/noetic_rpi_lane_yaml.png)
13. Close both **rqt\_rconfigure** and **turtlebot3\_autorace\_detect\_lane**.
14. Open terminal and use the command on `Remote PC`.

```
$ roslaunch turtlebot3_autorace_detect detect_lane.launch mode:=action

```
15. Check if the results come out correctly.

	* Open terminal and use the command on `Remote PC`.
```
$ roslaunch turtlebot3_autorace_driving turtlebot3_autorace_control_lane.launch

```

	* Open terminal and use the command on `Remote PC`.
```
$ roslaunch turtlebot3_bringup turtlebot3_robot.launch

```
16. After using the commands, TurtleBot3 will start to run.

## [Traffic Sign Detection](#traffic-sign-detection "#traffic-sign-detection")

TurtleBot3 can detect traffic signs using a node with `SIFT algorithm`, and perform programmed tasks while it drives on a built track. Follow the provided instructions to use Traffic sign detection.

**NOTE**: More edges in the traffic sign increase recognition results from SIFT.

1. Take pictures of traffic signs by using TurtleBot3’s camera and `rqt_image_view` node.
2. Edit the pictures using a photo editor that can be used in Linux OS.
3. Put TurtleBot3 on the lane. Traffic signes should be placed where TurtleBot3 can see them easily.

**NOTE**: Do not have TurtleBot3 run on the lane yet.
4. Execute rqt\_image\_view on `Remote PC`.

```
$ rqt_image_view

```
5. Select `/camera/image_compensated` topic in the select box. A screen will show the view from TurtleBot3.
6. Capture the picture from `Remote PC`，and edit it with a photo editor.
7. Place the edited picture to turtlebot3\_autorace package you’ve placed **/turtlebot3\_autorace/turtlebot3\_autorace\_detect/file/detect\_sign/** and rename it as you want. (Although, you should change the file name written in the source **detect\_sign.py** file, if you want to change the default file names.)
8. Open terminal and use the command on `Remote PC`.

```
$ roslaunch turtlebot3_autorace_${Autorace\_Misson}_detect turtlebot3_autorace_detect_sign.launch

```
9. Open terminal and use the command on `Remote PC`.

```
$ rqt_image_view

```
10. Select `/detect/image_traffic_sign/compressed` topic in the select box. A screen will show the result of traffic sign detection.

## [Traffic Sign Detection](#traffic-sign-detection "#traffic-sign-detection")

TurtleBot3 can detect various signs with the `SIFT` algorithm which compares the source image and the camera image, and perform programmed tasks while it drives.  

Follow the instructions below to test the traffic sign detection.

**NOTE**: More edges in the traffic sign increase recognition results from the SIFT algorithm.  

Please refer to the link below for related information.  

[https://docs.opencv.org/master/da/df5/tutorial\_py\_sift\_intro.html](https://docs.opencv.org/master/da/df5/tutorial_py_sift_intro.html "https://docs.opencv.org/master/da/df5/tutorial_py_sift_intro.html")

1. Open a new terminal and launch Autorace Gazebo simulation. The `roscore` will be automatically launched with the **roslaunch** command.

```
$ roslaunch turtlebot3_gazebo turtlebot3_autorace_2020.launch

```
2. Open a new terminal and launch the teleoperation node. Drive the TurtleBot3 along the lane and stop where traffic signes can be clearly seen by the camera.

```
$ roslaunch turtlebot3_teleop turtlebot3_teleop_key.launch

```
3. Open a new terminal and launch the rqt\_image\_view.

```
$ rqt_image_view

```
4. Select the `/camera/image_compensated` topic to display the camera image.
5. Capture each traffic sign from the `rqt_image_view` and crop unnecessary part of image. For the best performance, it is recommended to use original traffic sign images used in the track.
6. Save the images in the turtlebot3\_autorace\_detect package **/turtlebot3\_autorace\_2020/turtlebot3\_autorace\_detect/image/**. The file name should match with the name used in the source code.
	* `construction.png`, `intersection.png`, `left.png`, `right.png`, `parking.png`, `stop.png`, `tunnel.png` file names are used by default.
7. Open a new terminal and launch the intrinsic calibration node.

```
$ roslaunch turtlebot3_autorace_camera intrinsic_camera_calibration.launch

```
8. Open a new terminal and launch the extrinsic calibration node.

```
$ roslaunch turtlebot3_autorace_camera extrinsic_camera_calibration.launch

```
9. Open a new terminal and launch the traffic sign detection node.  

A specific mission for the ***mission*** argument must be selected among below.
	* `intersection`, `construction`, `parking`, `level_crossing`, `tunnel`

	```
	$ roslaunch turtlebot3_autorace_detect detect_sign.launch mission:=SELECT_MISSION

	```**NOTE**: Replace the `SELECT_MISSION` keyword with one of available options in the above.
10. Open a new terminal and launch the rqt image view plugin.

```
$ rqt_image_view

```
11. Select `/detect/image_traffic_sign/compressed` topic from the drop down list. A screen will display the result of traffic sign detection.

![](/assets/images/platform/turtlebot3/autonomous_driving/noetic_detect_intersection.png)

> 
> Detecting the Intersection sign when `mission:=intersection`
> 
> 
> 

![](/assets/images/platform/turtlebot3/autonomous_driving/noetic_detect_left.png)

> 
> Detecting the Left sign when `mission:=intersection`
> 
> 
> 

![](/assets/images/platform/turtlebot3/autonomous_driving/noetic_detect_right.png)

> 
> Detecting the Right sign when `mission:=intersection`
> 
> 
> 

![](/assets/images/platform/turtlebot3/autonomous_driving/noetic_detect_construction.png)

> 
> Detecting the Construction sign when `mission:=construction`
> 
> 
> 

![](/assets/images/platform/turtlebot3/autonomous_driving/noetic_detect_parking.png)

> 
> Detecting the Parking sign when `mission:=parking`
> 
> 
> 

![](/assets/images/platform/turtlebot3/autonomous_driving/noetic_detect_level_crossing.png)

> 
> Detecting the Level Crossing sign when `mission:=level_crossing`
> 
> 
> 

![](/assets/images/platform/turtlebot3/autonomous_driving/noetic_detect_tunnel.png)

> 
> Detecting the Tunnel sign when `mission:=tunnel`
> 
> 
>

## [Missions](#missions "#missions")

The AutoRace is a competition for autonomous driving robot platforms.  

To provide various conditions for a robot application development, the game provide structural regulation as less as possible. Provided open sources are based on ROS, and can be applied to this competition. The contents can be continually updated. Join the competition and show your skill.

**WARNING**: Be sure to read [Autonomous Driving](#autonomous-driving "#autonomous-driving") in order to start missions.

## [Missions](#missions "#missions")

The AutoRace is a competition for autonomous driving robot platforms.  

To provide various conditions for a robot application development, the game provide structural regulation as less as possible. Provided open sources are based on ROS, and can be applied to this competition. The contents can be continually updated. Join the competition and show your skill.

**WARNING**: Be sure to read [Autonomous Driving](#autonomous-driving "#autonomous-driving") in order to start missions.

### [Traffic Lights](#traffic-lights "#traffic-lights")

Traffic Light is the first mission of AutoRace. TurtleBot3 recognizes the traffic lights and starts the course.

#### [Traffic Lights Detection](#traffic-lights-detection "#traffic-lights-detection")

1. Open terminal and use the command on `Remote PC`.

```
$ export AUTO\_DT\_CALIB=calibration
$ roslaunch turtlebot3_autorace_traffic_light_detect turtlebot3_autorace_detect_traffic_light.launch

```
2. Excute rqt on `Remote PC`.

```
rqt

```
3. Select four topics: `/detect/image_red_light`, `/detect/image_yellow_light`, `/detect/image_green_light`, `/detect/image_traffic_light`.
4. Excute rqt\_reconfigure.

```
$ rosrun rqt_reconfigure rqt_reconfigure

```
5. Adjust parameters regarding traffic light topics to enhance the detection of traffic signs. The way of adjusting parameters is similar to step 5 at [Lane Detection](#lane-detection "#lane-detection").
6. Open **traffic\_light.yaml** file located at **turtlebot3\_autorace\_traffic\_light\_detect/param/traffic\_light/**.
7. Write modified values to the file and save.
8. Terminate both running rqt and rqt\_reconfigure in order to test, from the next step, the calibration whether or not it is successfully applied.
9. Use the command on `Remote PC`.

```
$ export AUTO\_DT\_CALIB=action
$ roslaunch turtlebot3_autorace_traffic_light_detect turtlebot3_autorace_detect_traffic_light.launch

```
10. Execute rqt\_image\_view.

```
$ rqt_image_view

```
11. See traffic light calibration is successfully applied.

#### [How to Run Traffic Light Mission](#how-to-run-traffic-light-mission "#how-to-run-traffic-light-mission")

**WARNING**: Be sure to read [Camera Calibration for Traffic Lights](#camera-calibration-for-traffic-lights "#camera-calibration-for-traffic-lights") before running the traffic light node.

1. Use the command on `Remote PC`.

```
$ export AUTO\_IN\_CALIB=action
$ export GAZEBO\_MODE=false
$ roslaunch turtlebot3_autorace_traffic_light_camera turtlebot3_autorace_intrinsic_camera_calibration.launch

```
2. Use the command on `Remote PC`.

```
$ export AUTO\_EX\_CALIB=action
$ export AUTO\_DT\_CALIB=action
$ roslaunch turtlebot3_autorace_traffic_light_core turtlebot3_autorace_core.launch

```
3. Use the command on `Remote PC`.

```
$ rostopic pub -1 /core/decided_mode std_msgs/UInt8 "data: 3"

```

### [Traffic Lights](#traffic-lights "#traffic-lights")

Traffic Light is the first mission of AutoRace. TurtleBot3 recognizes the traffic lights and starts the course.

##### [Traffic Lights Detection](#traffic-lights-detection "#traffic-lights-detection")

**NOTE**: In order to fix the traffic ligth to a specific color in Gazebo, you may modify the controlMission method in the `core_node_mission` file in the ***turtlebot3\_autorace\_2020/turtlebot3\_autorace\_core/nodes/*** directory.

1. Open a new terminal and launch Autorace Gazebo simulation. The `roscore` will be automatically launched with the **roslaunch** command.

```
$ roslaunch turtlebot3_gazebo turtlebot3_autorace_2020.launch

```
2. Open a new terminal and launch the intrinsic calibration node.

```
$ roslaunch turtlebot3_autorace_camera intrinsic_camera_calibration.launch

```
3. Open a new terminal and launch the extrinsic calibration node.

```
$ roslaunch turtlebot3_autorace_camera extrinsic_camera_calibration.launch

```
4. Open a new terminal and launch the traffic light detection node with a calibration option.

```
$ roslaunch turtlebot3_autorace_detect detect_traffic_light.launch mode:=calibration

```
5. Open a new terminal to execute the rqt. Open four `rqt_image_view` plugins.

```
$ rqt

```
6. Select four topics: `/detect/image_red_light`, `/detect/image_yellow_light`, `/detect/image_green_light`, `/detect/image_traffic_light`.

![](/assets/images/platform/turtlebot3/autonomous_driving/noetic_detect_traffic_light_green.png)

> 
> Detecting the Green light. The image on the right displays `/detect/image_green_light` topic.
> 
> 
> 

![](/assets/images/platform/turtlebot3/autonomous_driving/noetic_detect_traffic_light_yellow.png)

> 
> Detecting the Yellow light. The image on the right displays `/detect/image_yellow_light` topic.
> 
> 
> 

![](/assets/images/platform/turtlebot3/autonomous_driving/noetic_detect_traffic_light_red.png)

> 
> Detecting the Red light. The image on the right displays `/detect/image_red_light` topic.
> 
> 
>
7. Open a new terminal and excute rqt\_reconfigure.

```
$ rosrun rqt_reconfigure rqt_reconfigure

```
8. Select `detect_traffic_light` on the left column and adjust parameters properly so that the colors of the traffic light can be well detected.

![](/assets/images/platform/turtlebot3/autonomous_driving/noetic_traffic_light_reconfigure.png)

> 
> Traffic light reconfigure
> 
> 
>
9. Open the `traffic_light.yaml` file located at ***turtlebot3\_autorace\_detect/param/traffic\_light/***.

![](/assets/images/platform/turtlebot3/autonomous_driving/noetic_traffic_light_yaml.png)
10. Write modified values and save the file.

##### [Testing Traffic Light Detection](#testing-traffic-light-detection "#testing-traffic-light-detection")

1. Close all terminals or terminate them with `Ctrl` + `C`
2. Open a new terminal and launch Autorace Gazebo simulation. The `roscore` will be automatically launched with the **roslaunch** command.

```
$ roslaunch turtlebot3_gazebo turtlebot3_autorace_2020.launch

```
3. Open a new terminal and launch the intrinsic calibration node.

```
$ roslaunch turtlebot3_autorace_camera intrinsic_camera_calibration.launch

```
4. Open a new terminal and launch the extrinsic calibration node.

```
$ roslaunch turtlebot3_autorace_camera extrinsic_camera_calibration.launch

```
5. Open a new terminal and launch the traffic light detection node.

```
$ roslaunch turtlebot3_autorace_detect detect_traffic_light.launch

```
6. Open a new terminal and execute the rqt\_image\_view.

```
$ rqt_image_view

```
7. Check each topics: `/detect/image_red_light`, `/detect/image_yellow_light`, `/detect/image_green_light`.

##### [How to Run Traffic Light Mission](#how-to-run-traffic-light-mission "#how-to-run-traffic-light-mission")

**WARNING**: Please calibrate the color as described in the [Traffic Lights Detecion](#traffic-lights-detection "#traffic-lights-detection") section before running the traffic light mission.

1. Close all terminals or terminate them with `Ctrl` + `C`
2. Open a new terminal and launch Autorace Gazebo simulation. The `roscore` will be automatically launched with the **roslaunch** command.

```
$ roslaunch turtlebot3_gazebo turtlebot3_autorace_2020.launch

```
3. Open a new terminal and launch the intrinsic calibration node.

```
$ roslaunch turtlebot3_autorace_camera intrinsic_camera_calibration.launch

```
4. Open a new terminal and launch the autorace core node with a specific mission name.

```
$ roslaunch turtlebot3_autorace_core turtlebot3_autorace_core.launch mission:=traffic_light

```
5. Open a new terminal and enter the command below. This will prepare to run the traffic light mission by setting the `decided_mode` to `3`.

```
$ rostopic pub -1 /core/decided_mode std_msgs/UInt8 "data: 3"

```
6. Launch the Gazebo mission node.

```
$ roslaunch turtlebot3_autorace_core turtlebot3_autorace_mission.launch

```

### [Intersection](#intersection "#intersection")

Intersection is the second mission of AutoRace. TurtleBot3 detects a specific traffic sign (such as a curve sign) at the intersection course, and go to the given direction.

#### [How to Run Intersection Mission](#how-to-run-intersection-mission "#how-to-run-intersection-mission")

1. Use the command on `Remote PC`.

```
$ export AUTO\_IN\_CALIB=action
$ roslaunch turtlebot3_autorace_intersection_camera turtlebot3_autorace_intrinsic_camera_calibration.launch

```
2. Use the command on `Remote PC`.

```
$ export AUTO\_EX\_CALIB=action
$ export AUTO\_DT\_CALIB=action
$ roslaunch turtlebot3_autorace_intersection_core turtlebot3_autorace_core.launch

```
3. Use the command on `Remote PC`.

```
$ rostopic pub -1 /core/decided_mode std_msgs/UInt8 "data: 2"

```

### [Intersection](#intersection "#intersection")

Intersection is the second mission of AutoRace. TurtleBot3 must detect the directional sign at the intersection, and proceed to the directed path.

##### [How to Run Intersection Mission](#how-to-run-intersection-mission "#how-to-run-intersection-mission")

1. Close all terminals or terminate them with `Ctrl` + `C`
2. Open a new terminal and launch Autorace Gazebo simulation. The `roscore` will be automatically launched with the **roslaunch** command.

```
$ roslaunch turtlebot3_gazebo turtlebot3_autorace_2020.launch

```
3. Open a new terminal and launch the intrinsic calibration node.

```
$ roslaunch turtlebot3_autorace_camera intrinsic_camera_calibration.launch

```
4. Open a new terminal and launch the keyboard teleoperation node.  

Drive the TurtleBot3 along the lane and stop before the intersection traffic sign.

```
$ roslaunch turtlebot3_teleop turtlebot3_teleop_key.launch

```
5. Open a new terminal and launch the autorace core node with a specific mission name.

```
$ roslaunch turtlebot3_autorace_core turtlebot3_autorace_core.launch mission:=intersection

```
6. Open a new terminal and launch the Gazebo mission node.

```
$ roslaunch turtlebot3_autorace_core turtlebot3_autorace_mission.launch

```
7. Open a new terminal and enter the command below. This will prepare to run the intersection mission by setting the `decided_mode` to `2`.

```
$ rostopic pub -1 /core/decided_mode std_msgs/UInt8 "data: 2"

```

### [Construction](#construction "#construction")

Construction is the third mission of AutoRace. TurtleBot3 avoids constructions on the track while it is driving.

#### [How to Run Construction Mission](#how-to-run-construction-mission "#how-to-run-construction-mission")

1. Use the command on `Remote PC`.

```
$ export AUTO\_IN\_CALIB=action
$ export GAZEBO\_MODE=false
$ roslaunch turtlebot3_autorace_construction_camera turtlebot3_autorace_intrinsic_camera_calibration.launch

```
2. Use the command on `Remote PC`.

```
$ export AUTO\_EX\_CALIB=action
$ export AUTO\_DT\_CALIB=action
$ roslaunch turtlebot3_autorace_construction_core turtlebot3_autorace_core.launch

```
3. Use the command on `Remote PC`.

```
$ rostopic pub -1 /core/decided_mode std_msgs/UInt8 "data: 2"

```

### [Construction](#construction "#construction")

Construction is the third mission of TurtleBot3 AutoRace 2020. TurtleBot3 must avoid obstacles in the construction area.

##### [How to Run Construction Mission](#how-to-run-construction-mission "#how-to-run-construction-mission")

1. Close all terminals or terminate them with `Ctrl` + `C`
2. Open a new terminal and launch Autorace Gazebo simulation. The `roscore` will be automatically launched with the **roslaunch** command.

```
$ roslaunch turtlebot3_gazebo turtlebot3_autorace_2020.launch

```
3. Open a new terminal and launch the intrinsic calibration node.

```
$ roslaunch turtlebot3_autorace_camera intrinsic_camera_calibration.launch

```
4. Open a new terminal and launch the keyboard teleoperation node.  

Drive the TurtleBot3 along the lane and stop before the construction traffic sign.

```
$ roslaunch turtlebot3_teleop turtlebot3_teleop_key.launch

```
5. Open a new terminal and launch the autorace core node with a specific mission name.

```
$ roslaunch turtlebot3_autorace_core turtlebot3_autorace_core.launch mission:=construction

```
6. Open a new terminal and enter the command below. This will prepare to run the construction mission by setting the `decided_mode` to `2`.

```
$ rostopic pub -1 /core/decided_mode std_msgs/UInt8 "data: 2"

```

### [Parking](#parking "#parking")

Parking is the fourth mission of AutoRace. TurtleBot3 detects the parking sign, and park itself at a parking lot.

#### [How to Run Parking Mission](#how-to-run-parking-mission "#how-to-run-parking-mission")

1. Use the command on `Remote PC`.

```
$ export AUTO\_IN\_CALIB=action
$ export GAZEBO\_MODE=false
$ roslaunch turtlebot3_autorace_parking_camera turtlebot3_autorace_intrinsic_camera_calibration.launch

```
2. Use the command on `Remote PC`.

```
$ export AUTO\_EX\_CALIB=action
$ export AUTO\_DT\_CALIB=action
$ roslaunch turtlebot3_autorace_parking_core turtlebot3_autorace_core.launch

```
3. Use the command on `Remote PC`.

```
$ rostopic pub -1 /core/decided_mode std_msgs/UInt8 "data: 2"

```

### [Parking](#parking "#parking")

Parking is the fourth mission of TurtleBot3 AutoRace 2020. TurtleBot3 must detect the parking sign, and park at an empty parking spot.

##### [How to Run Parking Mission](#how-to-run-parking-mission "#how-to-run-parking-mission")

1. Close all terminals or terminate them with `Ctrl` + `C`
2. Open a new terminal and launch Autorace Gazebo simulation. The `roscore` will be automatically launched with the **roslaunch** command.

```
$ roslaunch turtlebot3_gazebo turtlebot3_autorace_2020.launch

```
3. Open a new terminal and launch the intrinsic calibration node.

```
$ roslaunch turtlebot3_autorace_camera intrinsic_camera_calibration.launch

```
4. Open a new terminal and launch the keyboard teleoperation node.  

Drive the TurtleBot3 along the lane and stop before the parking traffic sign.

```
$ roslaunch turtlebot3_teleop turtlebot3_teleop_key.launch

```
5. Open a new terminal and launch the autorace core node with a specific mission name.

```
$ roslaunch turtlebot3_autorace_core turtlebot3_autorace_core.launch mission:=parking

```
6. Open a new terminal and launch the Gazebo mission node.

```
$ roslaunch turtlebot3_autorace_core turtlebot3_autorace_mission.launch

```
7. Open a new terminal and enter the command below. This will prepare to run the parking mission by setting the `decided_mode` to `2`.

```
$ rostopic pub -1 /core/decided_mode std_msgs/UInt8 "data: 2"

```

### [Level Crossing](#level-crossing "#level-crossing")

Level Crossing is the fifth mission of AutoRace. When TurtleBot3 encounters the level crossing, it stops driving, and wait until the level crossing opens.

#### [Level Crossing Detection](#level-crossing-detection "#level-crossing-detection")

1. Use the command on `Remote PC`.

```
$ export AUTO\_DT\_CALIB=calibration
$ roslaunch turtlebot3_autorace_level_crossing_detect turtlebot3_autorace_detect_level.launch

```
2. Execute rqt

```
$ rqt

```
3. Select two topics: `/detect/image_level_color_filtered`, `/detect/image_level`
4. Execute rqt\_reconfigure.

```
$ rosrun rqt_reconfigure rqt_reconfigure

```
5. Select `/detect_level` and adjust parameters regarding Level Crossing topics to enhance the detection of the level crossing object. The way of adjusting parameters is similar to step 5 at [Lane Detection](#lane-detection "#lane-detection").
6. Open **level.yaml** located at **turtlebot3\_autorace\_stop\_bar\_detect/param/level/**.
7. Write modified values to the file and save.
8. Run a detect level lanuch file.

```
$ export AUTO\_DT\_CALIB=action
$ roslaunch turtlebot3_autorace_detect turtlebot3_autorace_detect_level.launch

```

#### [How to Run Level Crossing Mission](#how-to-run-level-crossing-mission "#how-to-run-level-crossing-mission")
9. Use the command on `Remote PC`.

```
$ export AUTO\_IN\_CALIB=action
$ export GAZEBO\_MODE=false
$ roslaunch turtlebot3_autorace_level_crossing_camera turtlebot3_autorace_intrinsic_camera_calibration.launch

```
10. Use the command on `Remote PC`.

```
$ export AUTO\_EX\_CALIB=action
$ export AUTO\_DT\_CALIB=action
$ roslaunch turtlebot3_autorace_level_crossing_core turtlebot3_autorace_core.launch

```
11. Use the command on `Remote PC`.

```
$ rostopic pub -1 /core/decided_mode std_msgs/UInt8 "data: 2"

```

### [Level Crossing](#level-crossing "#level-crossing")

Level Crossing is the fifth mission of TurtleBot3 AutoRace 2020. TurtleBot3 must detect the stop sign and wait until the crossing gate is lifted.

##### [Level Crossing Detection](#level-crossing-detection "#level-crossing-detection")

1. Close all terminals or terminate them with `Ctrl` + `C`
2. Open a new terminal and launch Autorace Gazebo simulation. The `roscore` will be automatically launched with the **roslaunch** command.

```
$ roslaunch turtlebot3_gazebo turtlebot3_autorace_2020.launch

```
3. Open a new terminal and launch the intrinsic calibration node.

```
$ roslaunch turtlebot3_autorace_camera intrinsic_camera_calibration.launch

```
4. Open a new terminal and launch the extrinsic calibration node.

```
$ roslaunch turtlebot3_autorace_camera extrinsic_camera_calibration.launch

```
5. Open a new terminal and launch the level crossing detection node with a calibration option.

```
$ roslaunch turtlebot3_autorace_detect detect_level_crossing.launch mode:=calibration

```
6. Open a new terminal and execute rqt.

```
$ rqt

```
7. Select two topics: `/detect/image_level_color_filtered/compressed`, `/detect/image_level/compressed`.
 ![](/assets/images/platform/turtlebot3/autonomous_driving/noetic_detect_level.png)
8. Excute rqt\_reconfigure.

```
$ rosrun rqt_reconfigure rqt_reconfigure

```
9. Adjust parameters in the `detect_level_crossing` in the left column to enhance the detection of crossing gate.
![](/assets/images/platform/turtlebot3/autonomous_driving/noetic_level_reconfigure.png)
10. Open `level.yaml` file located at ***turtlebot3\_autorace\_detect/param/level/***.

![](/assets/images/platform/turtlebot3/autonomous_driving/noetic_level_yaml.png)
11. Write modified values to the file and save.

##### Testing Level Crossing Detection

1. Close all terminals or terminate them with `Ctrl` + `C`
2. Open a new terminal and launch Autorace Gazebo simulation. The `roscore` will be automatically launched with the **roslaunch** command.

```
$ roslaunch turtlebot3_gazebo turtlebot3_autorace_2020.launch

```
3. Open a new terminal and launch the intrinsic calibration node.

```
$ roslaunch turtlebot3_autorace_camera intrinsic_camera_calibration.launch

```
4. Open a new terminal and launch the extrinsic calibration node.

```
$ roslaunch turtlebot3_autorace_camera extrinsic_camera_calibration.launch

```
5. Open a new terminal and launch the level crossing detection node.

```
$ roslaunch turtlebot3_autorace_detect detect_level_crossing.launch

```
6. Open a new terminal and execute the rqt\_image\_view.

```
$ rqt_image_view

```
7. Check the image topic: `/detect/image_level/compressed`.

##### [How to Run Level Crossing Mission](#how-to-run-level-crossing-mission "#how-to-run-level-crossing-mission")

1. Close all terminals or terminate them with `Ctrl` + `C`
2. Open a new terminal and launch Autorace Gazebo simulation. The `roscore` will be automatically launched with the **roslaunch** command.

```
$ roslaunch turtlebot3_gazebo turtlebot3_autorace_2020.launch

```
3. Open a new terminal and launch the intrinsic calibration node.

```
$ roslaunch turtlebot3_autorace_camera intrinsic_camera_calibration.launch

```
4. Open a new terminal and launch the keyboard teleoperation node.  

Drive the TurtleBot3 along the lane and stop before the stop traffic sign.

```
$ roslaunch turtlebot3_teleop turtlebot3_teleop_key.launch

```
5. Open a new terminal and launch the autorace core node with a specific mission name.

```
$ roslaunch turtlebot3_autorace_core turtlebot3_autorace_core.launch mission:=level_crossing

```
6. Open a new terminal and launch the Gazebo mission node.

```
$ roslaunch turtlebot3_autorace_core turtlebot3_autorace_mission.launch

```
7. Open a new terminal and enter the command below. This will prepare to run the level crossing mission by setting the `decided_mode` to `2`.

```
$ rostopic pub -1 /core/decided_mode std_msgs/UInt8 "data: 2"

```

### [Tunnel](#tunnel "#tunnel")

Tunnel is the sixth mission of AutoRace. TurtleBot3 passes the tunnel successfully.

#### [How to Run Tunnel Mission](#how-to-run-tunnel-mission "#how-to-run-tunnel-mission")

1. Use the command on `Remote PC`.

```
$ export AUTO\_IN\_CALIB=action
$ export GAZEBO\_MODE=false
$ roslaunch turtlebot3_autorace_tunnel_camera turtlebot3_autorace_intrinsic_camera_calibration.launch

```
2. Use the command on `Remote PC`.

```
$ export AUTO\_EX\_CALIB=action
$ export AUTO\_DT\_CALIB=action
$ export TURTLEBOT3\_MODEL=burger
$ roslaunch turtlebot3_autorace_tunnel_core turtlebot3_autorace_core.launch

```
3. Use the command on `Remote PC`.

```
$ rostopic pub -1 /core/decided_mode std_msgs/UInt8 "data: 2"

```

### [Tunnel](#tunnel "#tunnel")

Tunnel is the sixth mission of TurtleBot3 AutoRace 2020. TurtleBot3 must avoid obstacles in the unexplored tunnel and exit successfully.

##### [How to Run Tunnel Mission](#how-to-run-tunnel-mission "#how-to-run-tunnel-mission")

**NOTE**: Change the navigation parameters in the **turtlebot3/turtlebot3\_navigation/param/** file. If you slam and make a new map, Place the new map to turtlebot3\_autorace package you’ve placed **/turtlebot3\_autorace/turtlebot3\_autorace\_driving/maps/**.

1. Close all terminals or terminate them with `Ctrl` + `C`
2. Open a new terminal and launch Autorace Gazebo simulation. The `roscore` will be automatically launched with the **roslaunch** command.

```
$ roslaunch turtlebot3_gazebo turtlebot3_autorace_2020.launch

```
3. Open a new terminal and launch the intrinsic calibration node.

```
$ roslaunch turtlebot3_autorace_camera intrinsic_camera_calibration.launch

```
4. Open a new terminal and launch the keyboard teleoperation node.  

Drive the TurtleBot3 along the lane and stop before the tunnel traffic sign.

```
$ roslaunch turtlebot3_teleop turtlebot3_teleop_key.launch

```
5. Open a new terminal and launch the autorace core node with a specific mission name.

```
$ roslaunch turtlebot3_autorace_core turtlebot3_autorace_core.launch mission:=tunnel

```
6. Open a new terminal and enter the command below. This will prepare to run the tunnel mission by setting the `decided_mode` to `2`.

```
$ rostopic pub -1 /core/decided_mode std_msgs/UInt8 "data: 2"

```

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

