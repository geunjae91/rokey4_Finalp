document.addEventListener('DOMContentLoaded', function () {
  const form = document.getElementById('kt_modal_new_address_form');
  const modalBody = document.getElementById('cartItems');
  const orderModal = document.getElementById("orderModal");

  if (!form) {
    console.error("❌ 폼을 찾을 수 없습니다. ID를 확인하세요: #kt_modal_new_address_form");
    return;
  }

  if (!orderModal) {
    console.error("❌ 모달을 찾을 수 없습니다. ID를 확인하세요: #orderModal");
    return;
  }

  // 1️⃣ 모달이 열릴 때 장바구니 목록 불러오기
  orderModal.addEventListener("show.bs.modal", () => {
    fetch("/cart_items")
      .then(res => res.json())
      .then(data => {
        modalBody.innerHTML = "";
        if (data.length === 0) {
          modalBody.innerHTML = "<p>장바구니가 비어 있습니다.</p>";
        } else {
          data.forEach(item => {
            const row = document.createElement("div");
            row.className = "d-flex justify-content-between mb-2";
            row.innerHTML = `
              <span>${item.product_name}</span>
              <span>${item.quantity}개</span>
              <span>${item.price}</span>
            `;
            modalBody.appendChild(row);
          });
        }
      })
      .catch(err => {
        modalBody.innerHTML = "<p>장바구니 불러오기 실패</p>";
        console.error("🛒 장바구니 오류:", err);
      });
  });

  // 2️⃣ 주문 폼 제출 이벤트
  form.addEventListener('submit', function (e) {
    e.preventDefault();

    const formData = new FormData(form);

    // 3️⃣ 장바구니 데이터 추출
    const cartItems = [];
    document.querySelectorAll('#cartItems > div').forEach(row => {
      const spans = row.querySelectorAll('span');
      if (spans.length === 3) {
        cartItems.push({
          product_name: spans[0].innerText.trim(),
          quantity: parseInt(spans[1].innerText),
          price: parseFloat(spans[2].innerText)
        });
      }
    });

    // 4️⃣ cart_items 추가
    formData.append('cart_items', JSON.stringify(cartItems));

    // ✅ 디버깅용 로그
    console.log("📤 전송되는 주문 데이터:");
    for (let [key, value] of formData.entries()) {
      console.log(`${key}: ${value}`);
    }

    // 5️⃣ 주문 요청 전송
    fetch('/submit_order', {
      method: 'POST',
      body: formData,
    })
      .then(res => res.text())
      .then(result => {
        if (result.trim() === 'success') {
          alert('✅ 주문이 완료되었습니다.');
          const modalInstance = bootstrap.Modal.getInstance(orderModal);
          modalInstance.hide();
          form.reset();
        } else {
          alert('❌ 주문 실패: 서버 응답 오류');
        }
      })
      .catch(err => {
        console.error('❌ 주문 요청 실패:', err);
        alert('❌ 서버 오류가 발생했습니다.');
      });
  });
});
