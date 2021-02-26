// pass the map guid and call getmap API
getMap(guid: string) {
this.sub = this.restApiService.getMap(guid).subscribe(
  (result: any) => {
if (result) {
 this.mapData = result;
 this.base64Image = this.mapData.map; // this is the map data
 this.mapResolution = this.mapData.resolution;
 let imageBlob = this.b64toBlob(this.base64Image, "image/png", 1024); // call this method to start data > image process
 this.createImageFromBlob(imageBlob);
}
  },
  error => {
console.log(error);
throw error;
  });
}

/**
* Convert a base64 string in a Blob according to the data and contentType.
*
* @param b64Data {String} Pure base64 string without contentType
* @param contentType {String} the content type of the file i.e (image/jpeg - image/png - text/plain)
* @param sliceSize {Int} SliceSize to process the byteCharacters
* @see http://stackoverflow.com/questions/16245767/creating-a-blob-from-a-base64-string-in-javascript
* @return Blob
*/
b64toBlob(b64Data, contentType, sliceSize) {
contentType = contentType || '';
sliceSize = sliceSize || 512;

var byteCharacters = atob(b64Data);
var byteArrays = [];

for (var offset = 0; offset < byteCharacters.length; offset += sliceSize) {
 var slice = byteCharacters.slice(offset, offset + sliceSize);

 var byteNumbers = new Array(slice.length);
 for (var i = 0; i < slice.length; i++) {
byteNumbers[i] = slice.charCodeAt(i);
 }

 var byteArray = new Uint8Array(byteNumbers);

 byteArrays.push(byteArray);
}

var blob = new Blob(byteArrays, { type: contentType });
return blob;
}

createImageFromBlob(image: Blob) {
let reader = new FileReader();
reader.addEventListener("load", () => {
 this.mapImage = reader.result;
 this.confirmMapImage.emit(this.mapImage);
 this.setResolution.emit(this.mapResolution);
}, false);

if (image) {
 reader.readAsDataURL(image);
}
}