// Call the dataTables jQuery plugin
$(document).ready(function() {
  $('#dataTable').DataTable();
});

document.addEventListener("DOMContentLoaded", function () {
  fetch("/order_list")
    .then(res => res.json())
    .then(data => {
      const tbody = document.getElementById("orderTableBody");
      tbody.innerHTML = ""; // 초기화

      data.forEach(row => {
        const tr = document.createElement("tr");
        tr.innerHTML = `
          <td>${row.id}</td>
          <td>${row.name}</td>
          <td>${row.phone}</td>
          <td>${row.address}</td>
          <td>${row.product_name}</td>
          <td>${row.product_ea}</td>
        `;
        tbody.appendChild(tr);
      });

      // DataTables 플러그인 적용 (선택)
      if (window.$ && $.fn.DataTable) {
        $("#datatablesSimple").DataTable(); // 이미 초기화됐다면 생략
      }
    })
    .catch(err => {
      console.error("주문 목록 불러오기 실패:", err);
    });
});
